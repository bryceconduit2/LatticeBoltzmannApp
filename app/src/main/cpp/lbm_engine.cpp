#include <jni.h>
#include <android/bitmap.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

struct LBMEngine {
    int width, height;
    float uInlet = 0.12f;
    float uInletTarget = 0.12f;
    float omega = 1.0f / 0.55f;
    const float cs2 = 1.0f / 3.0f;
    const float SmagorinskyConstant = 0.16f;

    // Physical Constants
    const float dx = 0.0025f;     // 0.5m height / 200 cells (also 1.0m length / 400 cells)
    const float dt = 0.00002f;    // Time step to keep u_lb stable at high speed
    const float rhoAir = 1.225f;  // kg/m^3
    const float nuAir = 1.5e-5f;  // m^2/s (kinematic viscosity)

    float dragForceNewtons = 0.0f;

    std::vector<float> f;
    std::vector<float> fNew;
    std::vector<bool> obstacles;
    std::vector<float> velocityMag;
    std::vector<float> smoothedVelocityMag;

    const int cxs[9] = {0, 1, 0, -1, 0, 1, -1, -1, 1};
    const int cys[9] = {0, 0, 1, 0, -1, 1, 1, -1, -1};
    const float weights[9] = {4.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/36, 1.f/36, 1.f/36, 1.f/36};
    const int opposite[9] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

    LBMEngine(int w, int h) : width(w), height(h) {
        int size = width * height;
        f.resize(size * 9, 0.0f);
        fNew.resize(size * 9, 0.0f);
        obstacles.resize(size, false);
        velocityMag.resize(size, 0.0f);
        smoothedVelocityMag.resize(size, 0.0f);

        initFluid(uInlet);
    }

    void initFluid(float velocity) {
        float usqr = 1.5f * (velocity * velocity);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = (y * width + x) * 9;
                for (int i = 0; i < 9; i++) {
                    float cu = 3.0f * (static_cast<float>(cxs[i]) * velocity);
                    f[idx + i] = weights[i] * (1.0f + cu + 0.5f * cu * cu - usqr);
                }
            }
        }
        fNew = f;
    }

    void reset() {
        std::fill(obstacles.begin(), obstacles.end(), false);
        std::fill(velocityMag.begin(), velocityMag.end(), 0.0f);
        std::fill(smoothedVelocityMag.begin(), smoothedVelocityMag.end(), 0.0f);
        initFluid(uInlet);
    }

    void step() {
        // 0. Smooth inlet velocity to prevent numerical shocks
        uInlet = uInlet * 0.99f + uInletTarget * 0.01f;

        // Calculate physical omega based on air viscosity
        float nuLB = nuAir * dt / (dx * dx);
        omega = 1.0f / (3.0f * nuLB + 0.5f);

        // Reset drag counter for this step
        float localDragX = 0.0f;

        // 1. Streaming (Pull Method for Cache Efficiency)
#pragma omp parallel for default(none) shared(fNew, f, height, width, cxs, cys, opposite) reduction(+:localDragX)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int fBase = (y * width + x) * 9;
                for (int i = 0; i < 9; i++) {
                    int nx = x - cxs[i];
                    int ny = y - cys[i];

                    if (nx < 0) nx = width - 1; else if (nx >= width) nx = 0;
                    if (ny < 0) ny = height - 1; else if (ny >= height) ny = 0;

                    int sourceIdx = (ny * width + nx) * 9 + i;

                    // Momentum Exchange for Drag Calculation
                    if (obstacles[y * width + x] && !obstacles[ny * width + nx]) {
                        // Link from fluid (nx, ny) to solid (x, y)
                        // The particle that would have streamed into the solid bounces back
                        float f_to_solid = f[sourceIdx];
                        localDragX += f_to_solid * static_cast<float>(cxs[i]);
                    }

                    fNew[fBase + i] = f[sourceIdx];
                }
            }
        }

        // Convert lattice force to Newtons (assuming 1m depth)
        // F_phys = F_lb * rho * (dx^3 / dt^2)
        dragForceNewtons = localDragX * rhoAir * (dx * dx * dx) / (dt * dt);

        // 2. Collision with Regularization & Smagorinsky
#pragma omp parallel for default(none) shared(f, fNew, obstacles, velocityMag, width, height, uInlet, cs2, SmagorinskyConstant, omega, weights, cxs, cys, opposite)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                int fBase = idx * 9;

                if (obstacles[idx]) {
                    for (int i = 0; i < 9; i++) {
                        f[fBase + i] = fNew[fBase + opposite[i]];
                    }
                    velocityMag[idx] = 0.0f;
                } else {
                    float rho = 0.0f, ux = 0.0f, uy = 0.0f;
                    for (int i = 0; i < 9; i++) {
                        float fi = fNew[fBase + i];
                        rho += fi;
                        ux += fi * static_cast<float>(cxs[i]);
                        uy += fi * static_cast<float>(cys[i]);
                    }

                    // Numerical Safeguards: Clamp density
                    rho = std::max(0.1f, rho);
                    ux /= rho; uy /= rho;

                    if (x == 0) {
                        ux = uInlet; uy = 0.0f; rho = 1.0f;
                        velocityMag[idx] = ux;
                        float usqr_inlet = 1.5f * (ux * ux);
                        for (int i = 0; i < 9; i++) {
                            float cu = 3.0f * (static_cast<float>(cxs[i]) * ux);
                            f[fBase + i] = weights[i] * (1.0f + cu + 0.5f * cu * cu - usqr_inlet);
                        }
                    } else {
                        // Numerical Safeguards: Clamp velocity magnitude
                        float vMag = std::sqrt(ux * ux + uy * uy);
                        if (vMag > 0.5f) {
                            float scale = 0.5f / vMag;
                            ux *= scale; uy *= scale;
                            vMag = 0.5f;
                        }
                        velocityMag[idx] = vMag;

                        float usqr = 1.5f * (ux * ux + uy * uy);

                        // --- Regularization Logic ---
                        // Calculate non-equilibrium stress tensor (Pi)
                        float pixx = 0, pixy = 0, piyy = 0;
                        for (int i = 0; i < 9; i++) {
                            float fi_eq = rho * weights[i] * (1.0f + 3.0f*(cxs[i]*ux + cys[i]*uy) + 4.5f*(cxs[i]*ux + cys[i]*uy)*(cxs[i]*ux + cys[i]*uy) - usqr);
                            float fi_neq = fNew[fBase + i] - fi_eq;
                            pixx += fi_neq * cxs[i] * cxs[i];
                            pixy += fi_neq * cxs[i] * cys[i];
                            piyy += fi_neq * cys[i] * cys[i];
                        }

                        // --- Smagorinsky Model Logic ---
                        // Local strain rate S = sqrt(2 * S_ij * S_ij)
                        float pi_sq = pixx * pixx + 2.0f * pixy * pixy + piyy * piyy;
                        float S = std::sqrt(pi_sq) / (rho * 2.0f * cs2);
                        float tau_0 = 1.0f / omega;
                        float tau_t = 0.5f * (std::sqrt(tau_0 * tau_0 + 18.0f * SmagorinskyConstant * SmagorinskyConstant * S) - tau_0);
                        float omega_eff = 1.0f / (tau_0 + tau_t);

                        // --- Regularized Collision ---
                        for (int i = 0; i < 9; i++) {
                            float fi_eq = rho * weights[i] * (1.0f + 3.0f*(cxs[i]*ux + cys[i]*uy) + 4.5f*(cxs[i]*ux + cys[i]*uy)*(cxs[i]*ux + cys[i]*uy) - usqr);
                            float Qixx = cxs[i] * cxs[i] - cs2;
                            float Qixy = cxs[i] * cys[i];
                            float Qiyy = cys[i] * cys[i] - cs2;
                            // Add a small regularization factor (0.98) to muffle high-frequency non-equilibrium noise
                            float fi_neq_reg = 0.98f * (weights[i] / (2.0f * cs2 * cs2)) * (Qixx * pixx + 2.0f * Qixy * pixy + Qiyy * piyy);

                            f[fBase + i] = fi_eq + (1.0f - omega_eff) * fi_neq_reg;
                        }
                    }

                    // 3. Sponge Layer Damping (Right Outlet only)
                    if (x > width * 0.85) {
                        float spongeStart = static_cast<float>(width) * 0.85f;
                        float spongeWidth = static_cast<float>(width) * 0.15f;
                        // Cubic profile for smoother transition
                        float dist = (static_cast<float>(x) - spongeStart) / spongeWidth;
                        float alpha = dist * dist * dist * 0.2f;

                        float uInletSqr = 1.5f * (uInlet * uInlet);
                        for (int i = 0; i < 9; i++) {
                            // Target equilibrium at the outlet (rho=1.0, u=uInlet)
                            float cu = 3.0f * (static_cast<float>(cxs[i]) * uInlet);
                            float feqInlet = weights[i] * (1.0f + cu + 0.5f * cu * cu - uInletSqr);

                            // Relax f towards the outlet target to swallow pressure waves (ripples)
                            f[fBase + i] = f[fBase + i] * (1.0f - alpha) + feqInlet * alpha;
                        }
                    }
                }
            }
        }
    }
};



// Fast inline heatmap mapping (Blue -> Cyan -> Green -> Yellow -> Red)
inline uint32_t heatMapColor(float value) {
    value = std::max(0.0f, std::min(1.0f, value));
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
    // Android Bitmap ARGB_8888 is often RGBA in memory (0xAABBGGRR as uint32_t)
    return 0xFF000000 | (static_cast<uint32_t>(b * 255.0f) << 16) | (static_cast<uint32_t>(g * 255.0f) << 8) | static_cast<uint32_t>(r * 255.0f);
}

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

    // 1. Draw the new obstacle
    for (int y = std::max(0, cy - radius); y <= std::min(engine->height - 1, cy + radius); y++) {
        for (int x = std::max(0, cx - radius); x <= std::min(engine->width - 1, cx + radius); x++) {
            if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= radius * radius) {
                engine->obstacles[y * engine->width + x] = true;
            }
        }
    }
}


extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_stepAndRenderNative(JNIEnv *env, jobject thiz, jlong ptr, jobject bitmap, jint steps) {
    auto* engine = reinterpret_cast<LBMEngine*>(ptr);

    for (int i = 0; i < steps; i++) {
        engine->step();
    }

    void* pixels;
    if (AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) return;
    auto* bmpPixels = static_cast<uint32_t*>(pixels);

    float maxVel = engine->uInlet * 1.8f;
    int w = engine->width;
    int h = engine->height;

    // 1. Spatial 3x3 Smoothing Pass + Temporal Blending
#pragma omp parallel for default(none) shared(engine, w, h)
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;
            if (engine->obstacles[idx]) {
                engine->smoothedVelocityMag[idx] = 0.0f;
                continue;
            }

            // 3x3 Average Filter
            float sum = 0.0f;
            float count = 0.0f;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                        int nidx = ny * w + nx;
                        if (!engine->obstacles[nidx]) {
                            sum += engine->velocityMag[nidx];
                            count += 1.0f;
                        }
                    }
                }
            }
            float spatialAvg = (count > 0.0f) ? (sum / count) : 0.0f;

            // Temporal Damping (Exponential moving average)
            engine->smoothedVelocityMag[idx] = engine->smoothedVelocityMag[idx] * 0.7f + spatialAvg * 0.3f;
        }
    }

    // 2. Render Pass
    int totalPixels = w * h;
#pragma omp parallel for default(none) shared(engine, bmpPixels, maxVel, totalPixels)
    for (int i = 0; i < totalPixels; i++) {
        if (engine->obstacles[i]) {
            bmpPixels[i] = 0xFFFFFFFF;
        } else {
            bmpPixels[i] = heatMapColor(engine->smoothedVelocityMag[i] / maxVel);
        }
    }
    AndroidBitmap_unlockPixels(env, bitmap);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_resetSimulationNative(
        JNIEnv *env, jobject thiz, jlong ptr) {

    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    engine->reset();
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_setInletVelocityNative(
        JNIEnv *env, jobject thiz, jlong ptr, jfloat velocityPhys) {

    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    // Convert physical velocity (m/s) to lattice velocity
    float uInletLB = velocityPhys * engine->dt / engine->dx;
    // Limit velocity for numerical stability
    engine->uInletTarget = std::max(0.0f, std::min(0.45f, uInletLB));
}

extern "C" JNIEXPORT jfloat JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_getInletVelocityNative(
        JNIEnv *env, jobject thiz, jlong ptr) {

    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    // Convert back to physical m/s for UI
    return engine->uInlet * engine->dx / engine->dt;
}

extern "C" JNIEXPORT jfloat JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_getDragForceNative(
        JNIEnv *env, jobject thiz, jlong ptr) {

    auto* engine = reinterpret_cast<LBMEngine*>(ptr);
    return engine->dragForceNewtons;
}