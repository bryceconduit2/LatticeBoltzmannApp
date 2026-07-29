#include <jni.h>
#include <android/bitmap.h>
#include <android/log.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <omp.h> // Required for thread management
#include <chrono>

#define LOG_TAG "LBM_ENGINE"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

// Fast inline heatmap mapping (Blue -> Cyan -> Green -> Yellow -> Red)
inline uint32_t heatMapColor(float value) {
    // Use fmaxf and fminf for hardware-accelerated float math on ARM
    value = fmaxf(0.0f, fminf(1.0f, value));
    float r, g, b;
    if (value < 0.25f) { // Blue -> Cyan
        r = 0.0f; g = 4.0f * value; b = 1.0f;
    } else if (value < 0.5f) { // Cyan -> Green
        r = 0.0f; g = 1.0f; b = 1.0f - 4.0f * (value - 0.25f);
    } else if (value < 0.75f) { // Green -> Yellow
        r = 4.0f * (value - 0.5f); g = 1.0f; b = 0.0f;
    } else { // Yellow -> Red
        r = 1.0f; g = 1.0f - 4.0f * (value - 0.75f); b = 0.0f;
    }
    return 0xFF000000 | (static_cast<uint32_t>(b * 255.0f) << 16) | (static_cast<uint32_t>(g * 255.0f) << 8) | static_cast<uint32_t>(r * 255.0f);
}

struct LBMEngine {
    int width, height;
    float uInlet = 0.12f;
    float uInletTarget = 0.12f;
    float omega = 1.0f / 0.55f;
    const float cs2 = 1.0f / 3.0f;
    const float SmagorinskyConstant = 0.16f;

    // Physical Constants
    const float dx = 0.0025f;
    const float dt = 0.000005f;
    float rhoAir = 1.225f;
    float nuAir = 1.5e-5f;

    float dragForceNewtons = 0.0f;
    float dragCoefficient = 0.0f;
    float frontalArea = 0.0f;
    float dragSumAccumulator = 0.0f;
    int dragStepCount = 0;

    // Structure of Arrays (SoA) for better SIMD vectorization
    std::vector<float> f[9];
    std::vector<float> fNew[9];
    std::vector<uint8_t> obstacles;
    std::vector<float> velocityMag;
    std::vector<float> smoothedVelocityMag;

    uint32_t colorLUT[256];

    const float cxs[9] = {0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 1.0f, -1.0f, -1.0f, 1.0f};
    const float cys[9] = {0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f};
    const float weights[9] = {4.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/36, 1.f/36, 1.f/36, 1.f/36};
    const int opposite[9] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

    const float cxx[9] = {0, 1, 0, 1, 0, 1, 1, 1, 1};
    const float cyy[9] = {0, 0, 1, 0, 1, 1, 1, 1, 1};
    const float cxy[9] = {0, 0, 0, 0, 0, 1, -1, 1, -1};

    LBMEngine(int w, int h) : width(w), height(h) {
        // Precompute Color LUT for rendering
        for (int i = 0; i < 256; i++) {
            colorLUT[i] = heatMapColor(i / 255.0f);
        }

        // Android big.LITTLE Optimization:
        // Force OpenMP to use max 4 threads to avoid stalling on efficiency cores
        int numProcs = omp_get_num_procs();
        omp_set_num_threads(numProcs >= 8 ? 4 : numProcs);

        int size = width * height;
        for (int i = 0; i < 9; i++) {
            f[i].resize(size, 0.0f);
            fNew[i].resize(size, 0.0f);
        }
        obstacles.resize(size, 0);
        velocityMag.resize(size, 0.0f);
        smoothedVelocityMag.resize(size, 0.0f);

        initFluid(uInlet);
    }

    void initFluid(float velocity) {
        float usqr = 1.5f * (velocity * velocity);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                for (int i = 0; i < 9; i++) {
                    float cu = 3.0f * (cxs[i] * velocity);
                    f[i][idx] = weights[i] * (1.0f + cu + 0.5f * cu * cu - usqr);
                }
            }
        }
        for (int i = 0; i < 9; i++) fNew[i] = f[i];
    }

    void reset() {
        std::fill(obstacles.begin(), obstacles.end(), 0);
        std::fill(velocityMag.begin(), velocityMag.end(), 0.0f);
        std::fill(smoothedVelocityMag.begin(), smoothedVelocityMag.end(), 0.0f);
        initFluid(uInlet);
    }

    void step() {
        uInlet = uInlet * 0.99f + uInletTarget * 0.01f;

        float nuLB = nuAir * dt / (dx * dx);
        omega = 1.0f / (3.0f * nuLB + 0.5f);
        float tau_0 = 1.0f / omega;
        float inv2cs4 = 1.0f / (2.0f * cs2 * cs2);

        float localDragX = 0.0f;
        int stepMinY = height, stepMaxY = 0;

        const float smagCoeff = 18.0f * SmagorinskyConstant * SmagorinskyConstant;
        const float inv2cs2 = 0.5f / cs2;
        const float regFactor = 0.98f * inv2cs4;

        static const int CX[9] = { 0,  1,  0, -1,  0,  1, -1, -1,  1 };
        static const int CY[9] = { 0,  0,  1,  0, -1,  1,  1, -1, -1 };

        // Cache raw pointers for SoA access
        float* __restrict__ f_ptr[9];
        float* __restrict__ fn_ptr[9];
        for (int i = 0; i < 9; i++) {
            f_ptr[i] = f[i].data();
            fn_ptr[i] = fNew[i].data();
        }
        const uint8_t* __restrict__ obs = obstacles.data();
        float* __restrict__ vel = velocityMag.data();

        // ---------------------------------------------------------
        // PASS 1: INLET BOUNDARY (x = 0)
        // ---------------------------------------------------------
#pragma omp parallel for schedule(static) default(none) \
            shared(f_ptr, fn_ptr, obs, vel, width, height, uInlet, weights, cxs, opposite, CX, CY)
        for (int y = 0; y < height; y++) {
            int x = 0;
            int idx = y * width + x;

            if (obs[idx]) {
                for (int i = 0; i < 9; i++) {
                    int nx = x - CX[i];
                    int ny = y - CY[i];
                    if (nx < 0) nx = width - 1; else if (nx >= width) nx = 0;
                    if (ny < 0) ny = height - 1; else if (ny >= height) ny = 0;
                    fn_ptr[i][idx] = f_ptr[opposite[i]][ny * width + nx];
                }
                vel[idx] = 0.0f;
            } else {
                float ux = uInlet;
                float uy = 0.0f;
                vel[idx] = ux;
                float usqr_in = 1.5f * (ux * ux);

#pragma clang loop unroll(full)
                for (int i = 0; i < 9; i++) {
                    float cu = 3.0f * (cxs[i] * ux);
                    fn_ptr[i][idx] = weights[i] * (1.0f + cu + 0.5f * cu * cu - usqr_in);
                }
            }
        }

        // ---------------------------------------------------------
        // PASS 2: MAIN FLUID DOMAIN (x = 1 to width-1)
        // ---------------------------------------------------------
#pragma omp parallel for collapse(2) schedule(static) default(none) \
            shared(f_ptr, fn_ptr, obs, vel, width, height, cs2, smagCoeff, inv2cs2, \
                   tau_0, regFactor, weights, cxs, cys, opposite, cxx, cyy, cxy, CX, CY) \
            reduction(+:localDragX) reduction(min:stepMinY) reduction(max:stepMaxY)
        for (int y = 0; y < height; y++) {
            for (int x = 1; x < width; x++) {
                int idx = y * width + x;
                float local_f[9];

                if (obs[idx]) {
                    // --- OBSTACLE NODE PATH ---
#pragma clang loop unroll(full)
                    for (int i = 0; i < 9; i++) {
                        int nx = x - CX[i];
                        int ny = y - CY[i];
                        if (nx >= width) nx = 0;
                        if (ny < 0) ny = height - 1; else if (ny >= height) ny = 0;

                        int srcNeighborIdx = ny * width + nx;
                        float fi = f_ptr[i][srcNeighborIdx];
                        local_f[i] = fi;

                        if (!obs[srcNeighborIdx]) {
                            localDragX += 2.0f * fi * cxs[i];
                        }
                    }

#pragma clang loop unroll(full)
                    for (int i = 0; i < 9; ++i) {
                        fn_ptr[i][idx] = local_f[opposite[i]];
                    }
                    vel[idx] = 0.0f;

                    if (y < stepMinY) stepMinY = y;
                    if (y > stepMaxY) stepMaxY = y;

                } else {
                    // --- FLUID NODE PATH ---
#pragma clang loop unroll(full)
                    for (int i = 0; i < 9; i++) {
                        int nx = x - CX[i];
                        int ny = y - CY[i];
                        if (nx >= width) nx = 0;
                        if (ny < 0) ny = height - 1; else if (ny >= height) ny = 0;

                        local_f[i] = f_ptr[i][ny * width + nx];
                    }

                    float rho = local_f[0] + local_f[1] + local_f[2] + local_f[3] + local_f[4] +
                                local_f[5] + local_f[6] + local_f[7] + local_f[8];
                    rho = fmaxf(0.1f, rho);
                    float invRho = 1.0f / rho;

                    float ux = (local_f[1] - local_f[3] + local_f[5] - local_f[6] - local_f[7] + local_f[8]) * invRho;
                    float uy = (local_f[2] - local_f[4] + local_f[5] + local_f[6] - local_f[7] - local_f[8]) * invRho;

                    float vMag2 = ux * ux + uy * uy;
                    float vMag = sqrtf(vMag2);
                    if (vMag > 0.5f) {
                        float scale = 0.5f / vMag;
                        ux *= scale;
                        uy *= scale;
                        vMag = 0.5f;
                        vMag2 = 0.25f;
                    }
                    vel[idx] = vMag;
                    float usqr = 1.5f * vMag2;

                    float pixx = 0.0f, pixy = 0.0f, piyy = 0.0f;
                    float fi_eq[9];

#pragma clang loop unroll(full)
                    for (int i = 0; i < 9; i++) {
                        float ck_u = cxs[i] * ux + cys[i] * uy;
                        fi_eq[i] = rho * weights[i] * (1.0f + 3.0f * ck_u + 4.5f * ck_u * ck_u - usqr);
                        float fi_neq = local_f[i] - fi_eq[i];
                        pixx += fi_neq * cxx[i];
                        pixy += fi_neq * cxy[i];
                        piyy += fi_neq * cyy[i];
                    }

                    float S = sqrtf(pixx * pixx + 2.0f * pixy * pixy + piyy * piyy) * invRho * inv2cs2;
                    float tau_t = 0.5f * (sqrtf(tau_0 * tau_0 + smagCoeff * S) - tau_0);
                    float omega_eff = 1.0f / (tau_0 + tau_t);
                    float one_minus_omega = 1.0f - omega_eff;

#pragma clang loop unroll(full)
                    for (int i = 0; i < 9; i++) {
                        float Qpix = (cxx[i] - cs2) * pixx + 2.0f * cxy[i] * pixy + (cyy[i] - cs2) * piyy;
                        float fi_neq_reg = weights[i] * regFactor * Qpix;
                        fn_ptr[i][idx] = fi_eq[i] + one_minus_omega * fi_neq_reg;
                    }
                }
            }
        }

        dragSumAccumulator += localDragX;
        dragStepCount++;

        if (stepMaxY >= stepMinY) {
            frontalArea = static_cast<float>(stepMaxY - stepMinY + 1) * dx;
        } else {
            frontalArea = 0.0f;
        }

        for (int i = 0; i < 9; i++) f[i].swap(fNew[i]);
    }
};

extern "C" JNIEXPORT jlong JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_initEngine(JNIEnv *env, jobject thiz, jint width, jint height) {
    if (width <= 0 || height <= 0) return 0;
    return (jlong)(uintptr_t)new LBMEngine(width, height);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_destroyEngine(JNIEnv *env, jobject thiz, jlong ptr) {
    delete reinterpret_cast<LBMEngine*>(ptr);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_addObstacleNative(
        JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint radius) {

    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    int r2 = radius * radius;
    for (int y = std::max(0, cy - radius); y <= std::min(engine->height - 1, cy + radius); y++) {
        for (int x = std::max(0, cx - radius); x <= std::min(engine->width - 1, cx + radius); x++) {
            if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2) {
                engine->obstacles[y * engine->width + x] = 1;
            }
        }
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_addBoxObstacleNative(
        JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint size) {

    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    int halfSize = size / 2;
    for (int y = std::max(0, cy - halfSize); y <= std::min(engine->height - 1, cy + halfSize); y++) {
        for (int x = std::max(0, cx - halfSize); x <= std::min(engine->width - 1, cx + halfSize); x++) {
            engine->obstacles[y * engine->width + x] = 1;
        }
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_setDensityNative(
        JNIEnv *env, jobject thiz, jlong ptr, jfloat density) {
    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    engine->rhoAir = density;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_setViscosityNative(
        JNIEnv *env, jobject thiz, jlong ptr, jfloat viscosity) {
    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    engine->nuAir = viscosity;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_stepAndRenderNative(JNIEnv *env, jobject thiz, jlong ptr, jobject bitmap, jint steps) {
    auto* engine = reinterpret_cast<LBMEngine*>(ptr);

    engine->dragSumAccumulator = 0.0f;
    engine->dragStepCount = 0;

    auto startSim = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < steps; i++) {
        engine->step();
    }
    auto endSim = std::chrono::high_resolution_clock::now();

    if (engine->dragStepCount > 0) {
        float avgLatticeForceX = engine->dragSumAccumulator / static_cast<float>(engine->dragStepCount);

        // Physical Force (Newtons)
        float conversionFactor = engine->rhoAir * (engine->dx * engine->dx * engine->dx) / (engine->dt * engine->dt);
        engine->dragForceNewtons = fabsf(avgLatticeForceX) * conversionFactor;

        // Accurate Non-dimensional Drag Coefficient (Cd)
        // Calculated in lattice units to avoid unit errors: Cd = F_L / (0.5 * rho_L * u_L^2 * A_L)
        float uL = engine->uInlet;
        float rhoL = 1.0f; // Standard LBM density
        float areaL = engine->frontalArea / engine->dx; // Projected height in cells

        if (uL > 0.001f && areaL > 0.5f) {
            engine->dragCoefficient = fabsf(avgLatticeForceX) / (0.5f * rhoL * (uL * uL) * areaL);
        } else {
            engine->dragCoefficient = 0.0f;
        }
    }

    void* pixels;
    if (AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) return;
    auto* bmpPixels = static_cast<uint32_t*>(pixels);

    auto startRender = std::chrono::high_resolution_clock::now();

    float maxVel = engine->uInlet * 1.8f;
    int w = engine->width;
    int h = engine->height;

    const float invMaxVel = 255.0f / maxVel;
    const int w_minus_1 = w - 1;
    const int h_minus_1 = h - 1;

    static const float invCount[10] = {
            0.0f, 1.0f, 0.5f, 0.3333333f, 0.25f, 0.2f, 0.1666667f, 0.1428571f, 0.125f, 0.1111111f
    };

    const uint8_t* __restrict__ obs = engine->obstacles.data();
    const float* __restrict__ velMag = engine->velocityMag.data();
    float* __restrict__ smoothVel = engine->smoothedVelocityMag.data();
    const uint32_t* __restrict__ lut = engine->colorLUT;

#pragma omp parallel for default(none) shared(obs, velMag, smoothVel, bmpPixels, w, h, w_minus_1, h_minus_1, invMaxVel, invCount, lut) schedule(static)
    for (int y = 0; y < h; y++) {
        bool is_y_edge = (y == 0 || y == h_minus_1);
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;

            if (obs[idx]) {
                smoothVel[idx] = 0.0f;
                bmpPixels[idx] = 0xFFFFFFFF;
                continue;
            }

            float spatialAvg = 0.0f;
            bool is_x_edge = (x == 0 || x == w_minus_1);

            if (is_y_edge || is_x_edge) {
                float sum = 0.0f;
                int count = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    int ny = y + dy;
                    if (ny >= 0 && ny < h) {
                        for (int dx = -1; dx <= 1; dx++) {
                            int nx = x + dx;
                            if (nx >= 0 && nx < w) {
                                int nidx = ny * w + nx;
                                sum += velMag[nidx];
                                count += (1 - obs[nidx]);
                            }
                        }
                    }
                }
                spatialAvg = sum * invCount[count];
            } else {
                float sum = 0.0f;
                int count = 0;
                int top = idx - w;
                int bottom = idx + w;

                sum += velMag[top - 1]; count += (1 - obs[top - 1]);
                sum += velMag[top];     count += (1 - obs[top]);
                sum += velMag[top + 1]; count += (1 - obs[top + 1]);

                sum += velMag[idx - 1]; count += (1 - obs[idx - 1]);
                sum += velMag[idx];     count += (1 - obs[idx]);
                sum += velMag[idx + 1]; count += (1 - obs[idx + 1]);

                sum += velMag[bottom - 1]; count += (1 - obs[bottom - 1]);
                sum += velMag[bottom];     count += (1 - obs[bottom]);
                sum += velMag[bottom + 1]; count += (1 - obs[bottom + 1]);

                spatialAvg = sum * invCount[count];
            }

            float renderedV = smoothVel[idx] * 0.7f + spatialAvg * 0.3f;
            smoothVel[idx] = renderedV;

            // Branchless LUT mapping
            int colorIdx = (int)(renderedV * invMaxVel);
            colorIdx = std::max(0, std::min(255, colorIdx));
            bmpPixels[idx] = lut[colorIdx];
        }
    }
    AndroidBitmap_unlockPixels(env, bitmap);

    auto endRender = std::chrono::high_resolution_clock::now();

    auto simMs = std::chrono::duration_cast<std::chrono::milliseconds>(endSim - startSim).count();
    auto renderMs = std::chrono::duration_cast<std::chrono::milliseconds>(endRender - startRender).count();

    static int frameCounter = 0;
    if (++frameCounter % 60 == 0) {
        LOGI("Performance: %d steps in %lld ms (%.2f ms/step), Render: %lld ms",
             steps, simMs, (float)simMs / (steps > 0 ? steps : 1), renderMs);
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_resetSimulationNative(JNIEnv *env, jobject thiz, jlong ptr) {
    reinterpret_cast<LBMEngine*>(ptr)->reset();
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_setInletVelocityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat velocityPhys) {
    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    float uInletLB = velocityPhys * engine->dt / engine->dx;
    engine->uInletTarget = fmaxf(0.0f, fminf(0.45f, uInletLB));
}

extern "C" JNIEXPORT jfloat JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_getInletVelocityNative(JNIEnv *env, jobject thiz, jlong ptr) {
    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    return engine->uInlet * engine->dx / engine->dt;
}

extern "C" JNIEXPORT jfloat JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_getDragForceNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->dragForceNewtons;
}

extern "C" JNIEXPORT jfloat JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_getDragCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->dragCoefficient;
}

extern "C" JNIEXPORT jfloat JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_getDensityNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->rhoAir;
}

extern "C" JNIEXPORT jfloat JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_getViscosityNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->nuAir;
}

extern "C" JNIEXPORT jfloat JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_getDXNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->dx;
}