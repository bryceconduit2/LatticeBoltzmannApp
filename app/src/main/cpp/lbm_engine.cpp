#include <jni.h>
#include <android/bitmap.h>
#include <android/log.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <omp.h>
#include <chrono>

#define LOG_TAG "LBM_ENGINE"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

/**
 * Fast Heatmap Color Mapper
 */
inline uint32_t heatMapColor(float value) {
    value = fmaxf(0.0f, fminf(1.0f, value));
    float r, g, b;
    if (value < 0.25f) { r = 0.0f; g = 4.0f * value; b = 1.0f; }
    else if (value < 0.5f) { r = 0.0f; g = 1.0f; b = 1.0f - 4.0f * (value - 0.25f); }
    else if (value < 0.75f) { r = 4.0f * (value - 0.5f); g = 1.0f; b = 0.0f; }
    else { r = 1.0f; g = 1.0f - 4.0f * (value - 0.75f); b = 0.0f; }
    return 0xFF000000 | (static_cast<uint32_t>(b * 255.0f) << 16) | (static_cast<uint32_t>(g * 255.0f) << 8) | static_cast<uint32_t>(r * 255.0f);
}

struct ForceResult {
    float drag;
    float lift;
};

struct LBMEngine {
    enum VisualizationMode { VELOCITY = 0, PRESSURE = 1, TOTAL_PRESSURE = 2 };
    enum BoundaryMode { PERIODIC = 0, NO_SLIP = 1, FREE_SLIP = 2 };

    int width, height;
    float uInlet = 0.06f;
    float uInletTarget = 0.06f;
    float omega = 1.0f / 0.55f;
    const float cs2 = 1.0f / 3.0f;
    const float SmagorinskyConstant = 0.16f;

    const float dx = 0.0025f;
    const float dt = 0.000005f;
    float rhoAir = 1.225f;
    float nuAir = 1.5e-5f;

    float dragForceNewtons = 0.0f;
    float dragCoefficient = 0.0f;
    float liftForceNewtons = 0.0f;
    float liftCoefficient = 0.0f;
    float frontalArea = 0.0f;
    float horizontalSpan = 0.0f;
    unsigned long long totalSteps = 0;

    VisualizationMode vizMode = VELOCITY;
    BoundaryMode boundaryMode = PERIODIC;

    std::vector<float> f[9];
    std::vector<float> fNew[9];
    std::vector<float> fPost[9]; // Post-collision buffer
    std::vector<uint8_t> obstacles;
    std::vector<float> linkQ[9]; // BFL: distance q to the wall (0.0 if not a boundary link)
    std::vector<float> velocityMag;
    std::vector<float> visualizationSource;
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
        for (int i = 0; i < 256; i++) colorLUT[i] = heatMapColor(i / 255.0f);
        int numProcs = omp_get_num_procs();
        omp_set_num_threads(numProcs >= 8 ? 4 : numProcs);
        int size = width * height;
        for (int i = 0; i < 9; i++) {
            f[i].resize(size, 0.0f);
            fNew[i].resize(size, 0.0f);
            fPost[i].resize(size, 0.0f);
            linkQ[i].resize(size, 0.0f);
        }
        obstacles.resize(size, 0);
        velocityMag.resize(size, 0.0f);
        visualizationSource.resize(size, 0.0f);
        smoothedVelocityMag.resize(size, 0.0f);
        initFluid(uInlet);
    }

    void initFluid(float velocity) {
        float usqr = 1.5f * (velocity * velocity);
        for (int idx = 0; idx < width * height; idx++) {
            for (int i = 0; i < 9; i++) {
                float cu = 3.0f * (cxs[i] * velocity);
                f[i][idx] = weights[i] * (1.0f + cu + 0.5f * cu * cu - usqr);
            }
        }
        for (int i = 0; i < 9; i++) fNew[i] = f[i];
    }

    void reset() {
        std::fill(obstacles.begin(), obstacles.end(), 0);
        for (int i = 0; i < 9; i++) std::fill(linkQ[i].begin(), linkQ[i].end(), 0.0f);
        std::fill(velocityMag.begin(), velocityMag.end(), 0.0f);
        std::fill(visualizationSource.begin(), visualizationSource.end(), 0.0f);
        std::fill(smoothedVelocityMag.begin(), smoothedVelocityMag.end(), 0.0f);
        totalSteps = 0;
        initFluid(uInletTarget);
    }

    /**
     * Executes a single time step of the LBM algorithm.
     * Uses Bouzidi (BFL) Interpolated Bounce-Back for superior boundary accuracy.
     */
    ForceResult step() {
        totalSteps++;
        uInlet = uInlet * 0.99f + uInletTarget * 0.01f;
        float nuLB = nuAir * dt / (dx * dx);
        float tau_0 = 3.0f * nuLB + 0.5f;
        float inv2cs2 = 1.5f;
        float inv2cs4 = 4.5f;
        float regFactor = 0.98f * inv2cs4;
        float smagCoeff = 18.0f * SmagorinskyConstant * SmagorinskyConstant;

        float localDragX = 0.0f; float localLiftY = 0.0f;
        int stepMinY = height, stepMaxY = 0;
        int stepMinX = width, stepMaxX = 0;

        static const int CX[9] = { 0, 1, 0, -1, 0, 1, -1, -1, 1 };
        static const int CY[9] = { 0, 0, 1, 0, -1, 1, 1, -1, -1 };

        float* f_ptr[9]; float* fp_ptr[9]; float* fn_ptr[9];
        for (int i = 0; i < 9; i++) {
            f_ptr[i] = f[i].data();
            fp_ptr[i] = fPost[i].data();
            fn_ptr[i] = fNew[i].data();
        }
        const uint8_t* __restrict obs_ptr = obstacles.data();
        float* __restrict lq_ptr[9];
        for (int i = 0; i < 9; i++) lq_ptr[i] = linkQ[i].data();

        // --- PASS 1: Collision & Macroscopic Updates ---
#pragma omp parallel for collapse(2) schedule(static) default(none) \
    shared(f_ptr, fp_ptr, obs_ptr, velocityMag, visualizationSource, width, height, cs2, smagCoeff, inv2cs2, tau_0, regFactor, weights, cxs, cys, cxx, cyy, cxy, vizMode, uInlet)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                if (obs_ptr[idx]) continue;

                float local_f[9];
                for(int i=0; i<9; i++) local_f[i] = f_ptr[i][idx];

                float rho = local_f[0]+local_f[1]+local_f[2]+local_f[3]+local_f[4]+local_f[5]+local_f[6]+local_f[7]+local_f[8];
                rho = fmaxf(0.1f, rho); float invRho = 1.0f / rho;
                float ux = (local_f[1] - local_f[3] + local_f[5] - local_f[6] - local_f[7] + local_f[8]) * invRho;
                float uy = (local_f[2] - local_f[4] + local_f[5] + local_f[6] - local_f[7] - local_f[8]) * invRho;

                // Stability Limit
                float vMag2 = ux * ux + uy * uy; float vMag = sqrtf(vMag2);
                if (vMag > 0.45f) { float s = 0.45f / vMag; ux *= s; uy *= s; vMag = 0.45f; vMag2 = ux*ux+uy*uy; }
                velocityMag[idx] = vMag;

                // Collision
                float usqr = 1.5f * vMag2;
                float pixx = 0, pixy = 0, piyy = 0;
                float fi_eq[9];
                for (int i = 0; i < 9; i++) {
                    float cu = cxs[i] * ux + cys[i] * uy;
                    fi_eq[i] = rho * weights[i] * (1.0f + 3.0f * cu + 4.5f * cu * cu - usqr);
                    float fi_neq = local_f[i] - fi_eq[i];
                    pixx += fi_neq * cxx[i]; pixy += fi_neq * cxy[i]; piyy += fi_neq * cyy[i];
                }
                float S = sqrtf(pixx * pixx + 2.0f * pixy * pixy + piyy * piyy) * invRho * inv2cs2;
                float tau_eff = tau_0 + 0.5f * (sqrtf(tau_0 * tau_0 + smagCoeff * S) - tau_0);
                float one_minus_invTau = 1.0f - (1.0f / fmaxf(tau_eff, 0.501f));
                for (int i = 0; i < 9; i++) {
                    float Qpix = (cxx[i] - cs2) * pixx + 2.0f * cxy[i] * pixy + (cyy[i] - cs2) * piyy;
                    fp_ptr[i][idx] = fi_eq[i] + one_minus_invTau * weights[i] * regFactor * Qpix;
                }

                // Viz Source
                float ps = (rho - 1.0f) * cs2;
                float pressScale = (uInlet > 0.001f) ? (1.0f / (uInlet * uInlet)) : 100.0f;
                if (vizMode == VELOCITY) visualizationSource[idx] = vMag;
                else if (vizMode == PRESSURE) visualizationSource[idx] = 0.5f + ps * pressScale;
                else visualizationSource[idx] = 0.5f + (ps + 0.5f * rho * vMag2) * pressScale;
            }
        }

        // --- PASS 2: Streaming & BFL & Forces ---
#pragma omp parallel for collapse(2) schedule(static) default(none) \
    shared(fp_ptr, fn_ptr, obs_ptr, lq_ptr, width, height, opposite, CX, CY, cxs, cys, weights, uInlet, boundaryMode) \
    reduction(+:localDragX, localLiftY) reduction(min:stepMinY, stepMinX) reduction(max:stepMaxY, stepMaxX)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = y * width + x;
                if (obs_ptr[idx]) {
                    if (y < stepMinY) stepMinY = y; if (y > stepMaxY) stepMaxY = y;
                    if (x < stepMinX) stepMinX = x; if (x > stepMaxX) stepMaxX = x;
                    continue;
                }

                // Handle Inlet special (x=0)
                if (x == 0) {
                    float usqr = 1.5f * uInlet * uInlet;
                    for(int i=0; i<9; i++) {
                        float cu = 3.0f * cxs[i] * uInlet;
                        fn_ptr[i][idx] = weights[i] * (1.0f + cu + 0.5f * cu * cu - usqr);
                    }
                    continue;
                }

                for (int i = 0; i < 9; i++) {
                    int nx = x - CX[i]; int ny = y - CY[i];

                    // Boundary Conditions
                    if (nx < 0) nx = width - 1; else if (nx >= width) nx = 0;
                    if (ny < 0 || ny >= height) {
                        if (boundaryMode == PERIODIC) {
                            if (ny < 0) ny = height - 1; else ny = 0;
                        } else {
                            // Non-slip edge bounce back
                            fn_ptr[i][idx] = fp_ptr[opposite[i]][idx];
                            continue;
                        }
                    }

                    int srcIdx = ny * width + nx;
                    if (obs_ptr[srcIdx]) {
                        // BOUZIDI (BFL) INTERPOLATED BOUNCE-BACK
                        // link direction from idx to srcIdx was opposite[i].
                        float q = lq_ptr[opposite[i]][idx];
                        if (q <= 0.001f) q = 0.5f; // Fallback

                        if (q < 0.5f) {
                            // Wall is closer to fluid node
                            int nx2 = x + CX[i]; int ny2 = y + CY[i]; // node further in fluid
                            if (nx2 < 0) nx2 = width-1; else if (nx2 >= width) nx2 = 0;
                            if (ny2 < 0) ny2 = height-1; else if (ny2 >= height) ny2 = 0;
                            int idx2 = ny2 * width + nx2;

                            fn_ptr[i][idx] = 2.0f * q * fp_ptr[opposite[i]][idx] + (1.0f - 2.0f * q) * fp_ptr[opposite[i]][idx2];
                        } else {
                            // Wall is further from fluid node
                            fn_ptr[i][idx] = (1.0f / (2.0f * q)) * fp_ptr[opposite[i]][idx] + ((2.0f * q - 1.0f) / (2.0f * q)) * fn_ptr[opposite[i]][idx];
                        }

                        // Force using Momentum Exchange
                        localDragX += 2.0f * (fp_ptr[opposite[i]][idx] - weights[opposite[i]]) * cxs[opposite[i]];
                        localLiftY += 2.0f * (fp_ptr[opposite[i]][idx] - weights[opposite[i]]) * cys[opposite[i]];
                    } else {
                        // Standard Streaming
                        fn_ptr[i][idx] = fp_ptr[i][srcIdx];
                    }
                }
            }
        }

        if (stepMaxY >= stepMinY) frontalArea = static_cast<float>(stepMaxY - stepMinY + 1) * dx; else frontalArea = 0.0f;
        if (stepMaxX >= stepMinX) horizontalSpan = static_cast<float>(stepMaxX - stepMinX + 1) * dx; else horizontalSpan = 0.0f;
        for (int i = 0; i < 9; i++) f[i].swap(fNew[i]);
        return {localDragX, -localLiftY};
    }
};

extern "C" JNIEXPORT jlong JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_initEngine(JNIEnv *env, jobject thiz, jint width, jint height) {
    return (jlong)(uintptr_t)new LBMEngine(width, height);
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_destroyEngine(JNIEnv *env, jobject thiz, jlong ptr) {
    delete reinterpret_cast<LBMEngine*>(ptr);
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_addObstacleNative(JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint radius) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); int r2 = radius * radius; float R = (float)radius;
    for (int y = std::max(0, cy - radius); y <= std::min(e->height - 1, cy + radius); y++)
        for (int x = std::max(0, cx - radius); x <= std::min(e->width - 1, cx + radius); x++)
            if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2) e->obstacles[y * e->width + x] = 1;
    for (int y = std::max(0, cy - radius - 1); y <= std::min(e->height - 1, cy + radius + 1); y++) {
        for (int x = std::max(0, cx - radius - 1); x <= std::min(e->width - 1, cx + radius + 1); x++) {
            int idx = y * e->width + x; if (e->obstacles[idx]) continue;
            float dxf = (float)(x - cx); float dyf = (float)(y - cy);
            for (int i = 1; i < 9; i++) {
                int nx = x + (int)e->cxs[i]; int ny = y + (int)e->cys[i];
                if (nx < 0 || nx >= e->width || ny < 0 || ny >= e->height) continue;
                if (e->obstacles[ny * e->width + nx]) {
                    float vx = e->cxs[i]; float vy = e->cys[i];
                    float A = vx*vx + vy*vy; float B = 2.0f * (dxf*vx + dyf*vy); float C = dxf*dxf + dyf*dyf - R*R;
                    float det = B*B - 4.0f*A*C;
                    if (det >= 0) {
                        float q = (-B + sqrtf(det)) / (2.0f * A);
                        if (q > 0 && q < 1.0f) e->linkQ[i][idx] = q; else e->linkQ[i][idx] = 0.5f;
                    } else e->linkQ[i][idx] = 0.5f;
                }
            }
        }
    }
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_addBoxObstacleNative(JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint size) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); int h = size / 2;
    for (int y = std::max(0, cy - h); y <= std::min(e->height - 1, cy + h); y++)
        for (int x = std::max(0, cx - h); x <= std::min(e->width - 1, cx + h); x++)
            e->obstacles[y * e->width + x] = 1;
    for (int idx = 0; idx < e->width * e->height; idx++) {
        if (e->obstacles[idx]) continue;
        int x = idx % e->width; int y = idx / e->width;
        for (int i = 1; i < 9; i++) {
            int nx = x + (int)e->cxs[i]; int ny = y + (int)e->cys[i];
            if (nx >= 0 && nx < e->width && ny >= 0 && ny < e->height && e->obstacles[ny * e->width + nx]) e->linkQ[i][idx] = 0.5f;
        }
    }
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_addNacaAirfoilNative(
        JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint chord, jfloat m, jfloat p, jfloat t, jfloat angleDegrees) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); if (chord <= 0) return;
    float angleRad = -angleDegrees * (M_PI / 180.0f); float cosA = cosf(angleRad); float sinA = sinf(angleRad);
    int pad = chord / 2; int x_min = std::max(0, cx - pad); int x_max = std::min(e->width - 1, cx + chord + pad);
    int y_min = std::max(0, cy - chord); int y_max = std::min(e->height - 1, cy + chord);
    for (int py = y_min; py <= y_max; py++) {
        for (int px = x_min; px <= x_max; px++) {
            float dx_rel = (float)(px - cx); float dy_rel = (float)(py - cy);
            float x_loc = dx_rel * cosA + dy_rel * sinA; float y_loc = -dx_rel * sinA + dy_rel * cosA;
            float x_f = x_loc / (float)chord;
            if (x_f >= 0.0f && x_f <= 1.0f) {
                float yt = 5.0f * t * (0.2969f * sqrtf(x_f) - 0.1260f * x_f - 0.3516f * x_f * x_f + 0.2843f * powf(x_f, 3) - 0.1015f * powf(x_f, 4));
                float yc = 0.0f;
                if (p > 0.001f) {
                    if (x_f <= p) yc = (m / (p * p)) * (2.0f * p * x_f - x_f * x_f);
                    else yc = (m / powf(1.0f - p, 2)) * ((1.0f - 2.0f * p) + 2.0f * p * x_f - x_f * x_f);
                }
                if (fabsf(y_loc + yc * chord) <= yt * chord) e->obstacles[py * e->width + px] = 1;
            }
        }
    }
    for (int idx = 0; idx < e->width * e->height; idx++) {
        if (e->obstacles[idx]) continue;
        int x = idx % e->width; int y = idx / e->width;
        if (x < x_min || x > x_max || y < y_min || y > y_max) continue;
        for (int i = 1; i < 9; i++) {
            int nx = x + (int)e->cxs[i]; int ny = y + (int)e->cys[i];
            if (nx >= 0 && nx < e->width && ny >= 0 && ny < e->height && e->obstacles[ny * e->width + nx]) e->linkQ[i][idx] = 0.5f;
        }
    }
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setDensityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat d) { reinterpret_cast<LBMEngine*>(ptr)->rhoAir = d; }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setViscosityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat v) { reinterpret_cast<LBMEngine*>(ptr)->nuAir = v; }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_stepAndRenderNative(JNIEnv *env, jobject thiz, jlong ptr, jobject bitmap, jint steps) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); double dragAcc = 0.0; double liftAcc = 0.0;
    for (int i = 0; i < steps; i++) { ForceResult res = e->step(); dragAcc += (double)res.drag; liftAcc += (double)res.lift; }
    if (steps > 0) {
        float avgD = (float)(dragAcc / steps); float avgL = (float)(liftAcc / steps);
        float conv = e->rhoAir * (e->dx * e->dx * e->dx) / (e->dt * e->dt);
        e->dragForceNewtons = fabsf(avgD) * conv; e->liftForceNewtons = avgL * conv;
        float uL = e->uInlet; float refL = fmaxf(e->frontalArea, e->horizontalSpan) / e->dx;
        float dynP = 0.5f * (uL * uL);
        if (uL > 0.001f && refL > 0.5f) { e->dragCoefficient = fabsf(avgD) / (dynP * refL); e->liftCoefficient = avgL / (dynP * refL); }
        else { e->dragCoefficient = 0; e->liftCoefficient = 0; }
    }
    void* pixels; if (AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) return;
    auto* bmp = static_cast<uint32_t*>(pixels);
    int w = e->width; int h = e->height; float invMaxV = 255.0f / (e->uInlet * 1.8f);
    static const float invC[10] = { 0, 1.0f, 0.5f, 0.3333333f, 0.25f, 0.2f, 0.1666667f, 0.1428571f, 0.125f, 0.1111111f };
    const uint8_t* obs = e->obstacles.data(); const float* src = e->visualizationSource.data();
    float* sm = e->smoothedVelocityMag.data(); uint32_t* lut = e->colorLUT;
#pragma omp parallel for default(none) shared(obs, src, sm, bmp, w, h, invMaxV, invC, lut, e) schedule(static)
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y * w + x; if (obs[idx]) { sm[idx] = 0; bmp[idx] = 0xFFFFFFFF; continue; }
            float sum = 0; int count = 0;
            if (y == 0 || y == h-1 || x == 0 || x == w-1) {
                for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
                    int ny = y + dy; int nx = x + dx;
                    if (ny >= 0 && ny < h && nx >= 0 && nx < w) { sum += src[ny * w + nx]; count += (1 - obs[ny * w + nx]); }
                }
            } else {
                int t = idx - w; int b = idx + w;
                sum = src[t-1]+src[t]+src[t+1]+src[idx-1]+src[idx]+src[idx+1]+src[b-1]+src[b]+src[b+1];
                count = (1-obs[t-1])+(1-obs[t])+(1-obs[t+1])+(1-obs[idx-1])+(1-obs[idx])+(1-obs[idx+1])+(1-obs[b-1])+(1-obs[b])+(1-obs[b+1]);
            }
            float rv = sm[idx] * 0.7f + (sum * invC[count]) * 0.3f; sm[idx] = rv;
            int ci = (int)(rv * ((e->vizMode == LBMEngine::VELOCITY) ? invMaxV : 255.0f));
            bmp[idx] = lut[std::max(0, std::min(255, ci))];
        }
    }
    AndroidBitmap_unlockPixels(env, bitmap);
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_resetSimulationNative(JNIEnv *env, jobject thiz, jlong ptr) { reinterpret_cast<LBMEngine*>(ptr)->reset(); }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setInletVelocityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat v) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); float uL = v * e->dt / e->dx; e->uInletTarget = fmaxf(0.0f, fminf(0.45f, uL));
}
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getInletVelocityNative(JNIEnv *env, jobject thiz, jlong ptr) { auto* e = reinterpret_cast<LBMEngine*>(ptr); return e->uInlet * e->dx / e->dt; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDragForceNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->dragForceNewtons; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDragCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->dragCoefficient; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getLiftForceNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->liftForceNewtons; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getLiftCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->liftCoefficient; }
extern "C" JNIEXPORT jlong JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getTotalStepsNative(JNIEnv *env, jobject thiz, jlong ptr) { return (jlong)reinterpret_cast<LBMEngine*>(ptr)->totalSteps; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDensityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->rhoAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getViscosityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->nuAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDXNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->dx; }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setVisualizationModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint m) { reinterpret_cast<LBMEngine*>(ptr)->vizMode = static_cast<LBMEngine::VisualizationMode>(m); }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setBoundaryModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint mode) {
    reinterpret_cast<LBMEngine*>(ptr)->boundaryMode = static_cast<LBMEngine::BoundaryMode>(mode);
}
