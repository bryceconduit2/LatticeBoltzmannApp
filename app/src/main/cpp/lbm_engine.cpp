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
 * Precomputed equilibrium weights
 */
const float W0 = 4.0f/9.0f;
const float W1 = 1.0f/9.0f;
const float W2 = 1.0f/36.0f;

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
    float smoothedCd = 0.0f;
    float smoothedCl = 0.0f;
    int numThreads = 4;
    int activeCores = 0;
    float frontalArea = 0.0f;
    float horizontalSpan = 0.0f;
    unsigned long long totalSteps = 0;

    VisualizationMode vizMode = VELOCITY;
    BoundaryMode boundaryMode = PERIODIC;

    std::vector<float> f[9];
    std::vector<float> fNew[9];
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
        numThreads = (numProcs >= 8 ? 4 : numProcs);
        int size = width * height;
        for (int i = 0; i < 9; i++) {
            f[i].resize(size, 0.0f);
            fNew[i].resize(size, 0.0f);
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
        smoothedCd = 0.0f;
        smoothedCl = 0.0f;
        initFluid(uInletTarget);
    }

    /**
     * Executes a single time step of the LBM algorithm.
     * Fused Single-Pass Kernel: Pull-Streaming + Regularized BGK + Smagorinsky + BFL.
     * This drastically reduces memory bandwidth requirements.
     */
    ForceResult step() {
        totalSteps++;
        uInlet = uInlet * 0.99f + uInletTarget * 0.01f;
        const float nuLB = nuAir * dt / (dx * dx);
        const float tau_0 = 3.0f * nuLB + 0.5f;
        const float inv2cs2 = 1.5f;
        const float inv2cs4 = 4.5f;
        const float regFactor = 0.98f * inv2cs4;
        const float smagCoeff = 18.0f * SmagorinskyConstant * SmagorinskyConstant;

        float localDragX = 0.0f; float localLiftY = 0.0f;
        int stepMinY = height, stepMaxY = 0;
        int stepMinX = width, stepMaxX = 0;

        static const int CX[9] = { 0, 1, 0, -1, 0, 1, -1, -1, 1 };
        static const int CY[9] = { 0, 0, 1, 0, -1, 1, 1, -1, -1 };

        float* f_ptr[9]; float* fn_ptr[9];
        for (int i = 0; i < 9; i++) {
            f_ptr[i] = f[i].data();
            fn_ptr[i] = fNew[i].data();
        }
        const uint8_t* __restrict obs_ptr = obstacles.data();
        float* __restrict lq_ptr[9];
        for (int i = 0; i < 9; i++) lq_ptr[i] = linkQ[i].data();

#pragma omp parallel for collapse(2) schedule(static) default(none) \
    shared(f_ptr, fn_ptr, obs_ptr, lq_ptr, velocityMag, visualizationSource, width, height, cs2, smagCoeff, inv2cs2, tau_0, regFactor, weights, CX, CY, opposite, vizMode, uInlet, numThreads, boundaryMode, W0, W1, W2) \
    reduction(+:localDragX, localLiftY) reduction(min:stepMinY, stepMinX) reduction(max:stepMaxY, stepMaxX) \
    num_threads(numThreads)
        for (int y = 0; y < height; y++) {
            if (omp_get_thread_num() == 0) activeCores = omp_get_num_threads();
            const bool y_edge = (y == 0 || y == height - 1);

            for (int x = 0; x < width; x++) {
                const int idx = y * width + x;
                if (obs_ptr[idx]) {
                    if (y < stepMinY) stepMinY = y; if (y > stepMaxY) stepMaxY = y;
                    if (x < stepMinX) stepMinX = x; if (x > stepMaxX) stepMaxX = x;
                    continue;
                }

                // 1. PULL STREAMING + BOUNDARY HANDLING
                float local_f[9];

                // x=0 is the Inlet
                if (x == 0) {
                    const float u = uInlet;
                    const float term1 = 1.0f - 1.5f * u * u;
                    const float u3 = 3.0f * u;
                    const float u45 = 4.5f * u * u;
                    local_f[0] = weights[0] * term1;
                    local_f[1] = weights[1] * (term1 + u3 + u45);
                    local_f[2] = weights[2] * term1;
                    local_f[3] = weights[3] * (term1 - u3 + u45);
                    local_f[4] = weights[4] * term1;
                    local_f[5] = weights[5] * (term1 + u3 + u45);
                    local_f[6] = weights[6] * (term1 - u3 + u45);
                    local_f[7] = weights[7] * (term1 - u3 + u45);
                    local_f[8] = weights[8] * (term1 + u3 + u45);
                } else {
                    const bool is_interior = (!y_edge && x < width - 1);
                    for (int i = 0; i < 9; i++) {
                        int nx, ny;
                        if (is_interior) {
                            nx = x - CX[i]; ny = y - CY[i];
                        } else {
                            nx = x - CX[i]; ny = y - CY[i];
                            if (nx < 0) nx = width - 1; else if (nx >= width) nx = 0;
                            if (ny < 0 || ny >= height) {
                                if (boundaryMode == PERIODIC) { if (ny < 0) ny = height - 1; else ny = 0; }
                                else { local_f[i] = f_ptr[opposite[i]][idx]; continue; }
                            }
                        }

                        const int srcIdx = ny * width + nx;
                        if (obs_ptr[srcIdx]) {
                            // BFL Interpolated Bounce-Back
                            float q = lq_ptr[opposite[i]][idx];
                            if (q <= 0.001f) q = 0.5f;
                            if (q < 0.5f) {
                                int nx2 = x + CX[i]; int ny2 = y + CY[i];
                                if (nx2 < 0) nx2 = width-1; else if (nx2 >= width) nx2 = 0;
                                if (ny2 < 0) ny2 = height-1; else if (ny2 >= height) ny2 = 0;
                                local_f[i] = 2.0f * q * f_ptr[opposite[i]][idx] + (1.0f - 2.0f * q) * f_ptr[opposite[i]][ny2 * width + nx2];
                            } else {
                                float inv2q = 1.0f / (2.0f * q);
                                local_f[i] = inv2q * f_ptr[opposite[i]][idx] + (1.0f - inv2q) * fn_ptr[opposite[i]][idx]; // Use partial stream from current pass
                            }
                            localDragX += 2.0f * (f_ptr[opposite[i]][idx] - weights[opposite[i]]) * cxs[opposite[i]];
                            localLiftY += 2.0f * (f_ptr[opposite[i]][idx] - weights[opposite[i]]) * cys[opposite[i]];
                        } else {
                            local_f[i] = f_ptr[i][srcIdx];
                        }
                    }
                }

                // 2. MACROSCOPIC MOMENTS
                const float rho = local_f[0] + local_f[1] + local_f[2] + local_f[3] + local_f[4] + local_f[5] + local_f[6] + local_f[7] + local_f[8];
                const float invRho = 1.0f / fmaxf(0.1f, rho);
                float ux = (local_f[1] - local_f[3] + local_f[5] - local_f[6] - local_f[7] + local_f[8]) * invRho;
                float uy = (local_f[2] - local_f[4] + local_f[5] + local_f[6] - local_f[7] - local_f[8]) * invRho;

                float vMag2 = ux * ux + uy * uy; float vMag = sqrtf(vMag2);
                if (vMag > 0.45f) { float s = 0.45f / vMag; ux *= s; uy *= s; vMag = 0.45f; vMag2 = 0.2025f; }
                velocityMag[idx] = vMag;

                // 3. COLLISION (Regularized BGK + Smagorinsky)
                const float one_minus_15u2 = 1.0f - 1.5f * vMag2;
                const float row_w_9 = rho * W1;
                const float row_w_36 = rho * W2;
                const float ux3 = 3.0f * ux; const float uy3 = 3.0f * uy;
                const float ux9 = 4.5f * ux * ux; const float uy9 = 4.5f * uy * uy;
                const float uxy9 = 9.0f * ux * uy;

                float feq[9];
                feq[0] = rho * W0 * one_minus_15u2;
                feq[1] = row_w_9 * (one_minus_15u2 + ux3 + ux9);
                feq[2] = row_w_9 * (one_minus_15u2 + uy3 + uy9);
                feq[3] = row_w_9 * (one_minus_15u2 - ux3 + ux9);
                feq[4] = row_w_9 * (one_minus_15u2 - uy3 + uy9);
                const float common_diag = one_minus_15u2 + ux9 + uy9;
                feq[5] = row_w_36 * (common_diag + ux3 + uy3 + uxy9);
                feq[6] = row_w_36 * (common_diag - ux3 + uy3 - uxy9);
                feq[7] = row_w_36 * (common_diag - ux3 - uy3 + uxy9);
                feq[8] = row_w_36 * (common_diag + ux3 - uy3 - uxy9);

                float pixx = (local_f[1]-feq[1]) + (local_f[3]-feq[3]) + (local_f[5]-feq[5]) + (local_f[6]-feq[6]) + (local_f[7]-feq[7]) + (local_f[8]-feq[8]);
                float piyy = (local_f[2]-feq[2]) + (local_f[4]-feq[4]) + (local_f[5]-feq[5]) + (local_f[6]-feq[6]) + (local_f[7]-feq[7]) + (local_f[8]-feq[8]);
                float pixy = (local_f[5]-feq[5]) - (local_f[6]-feq[6]) + (local_f[7]-feq[7]) - (local_f[8]-feq[8]);

                const float S = sqrtf(pixx * pixx + 2.0f * pixy * pixy + piyy * piyy) * invRho * inv2cs2;
                const float tau_eff = tau_0 + 0.5f * (sqrtf(tau_0 * tau_0 + smagCoeff * S) - tau_0);
                const float one_minus_invTau = 1.0f - (1.0f / fmaxf(tau_eff, 0.501f));
                const float common_reg = one_minus_invTau * regFactor;

                const float reg_pixx = common_reg * (pixx * (1.0f - cs2) + piyy * (0.0f - cs2));
                const float reg_piyy = common_reg * (pixx * (0.0f - cs2) + piyy * (1.0f - cs2));
                const float reg_pixy = common_reg * 2.0f * pixy;

                fn_ptr[0][idx] = feq[0] + W0 * common_reg * (-cs2 * (pixx + piyy));
                fn_ptr[1][idx] = feq[1] + W1 * reg_pixx;
                fn_ptr[2][idx] = feq[2] + W1 * reg_piyy;
                fn_ptr[3][idx] = feq[3] + W1 * reg_pixx;
                fn_ptr[4][idx] = feq[4] + W1 * reg_piyy;
                fn_ptr[5][idx] = feq[5] + W2 * (reg_pixx + reg_pixy + reg_piyy);
                fn_ptr[6][idx] = feq[6] + W2 * (reg_pixx - reg_pixy + reg_piyy);
                fn_ptr[7][idx] = feq[7] + W2 * (reg_pixx + reg_pixy + reg_piyy);
                fn_ptr[8][idx] = feq[8] + W2 * (reg_pixx - reg_pixy + reg_piyy);

                if (std::isnan(fn_ptr[0][idx])) { for(int i=0; i<9; i++) fn_ptr[i][idx] = weights[i]; }

                const float ps = (rho - 1.0f) * cs2;
                const float pressScale = (uInlet > 0.001f) ? (1.0f / (uInlet * uInlet)) : 100.0f;
                if (vizMode == VELOCITY) visualizationSource[idx] = vMag;
                else if (vizMode == PRESSURE) visualizationSource[idx] = 0.5f + ps * pressScale;
                else visualizationSource[idx] = 0.5f + (ps + 0.5f * rho * vMag2) * pressScale;
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
        float uL = e->uInlet;        float refL = fmaxf(e->frontalArea, e->horizontalSpan) / e->dx;
        float dynP = 0.5f * (uL * uL);
        if (uL > 0.001f && refL > 0.5f) {
            float instantCd = fabsf(avgD) / (dynP * refL);
            float instantCl = avgL / (dynP * refL);
            e->dragCoefficient = instantCd;
            e->liftCoefficient = instantCl;

            // EMA Damping (Alpha=0.02 for ~50 frame stability window)
            // This dampens the Kármán vortex oscillations for the UI readout
            e->smoothedCd = e->smoothedCd * 0.98f + instantCd * 0.02f;
            e->smoothedCl = e->smoothedCl * 0.98f + instantCl * 0.02f;
        } else {
            e->dragCoefficient = 0; e->liftCoefficient = 0;
            e->smoothedCd = 0; e->smoothedCl = 0;
        }
    }
    void* pixels; if (AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) return;
    auto* bmp = static_cast<uint32_t*>(pixels);
    int w = e->width; int h = e->height; float invMaxV = 255.0f / (e->uInlet * 1.8f);
    static const float invC[10] = { 0, 1.0f, 0.5f, 0.3333333f, 0.25f, 0.2f, 0.1666667f, 0.1428571f, 0.125f, 0.1111111f };
    const uint8_t* obs = e->obstacles.data(); const float* src = e->visualizationSource.data();
    float* sm = e->smoothedVelocityMag.data(); uint32_t* lut = e->colorLUT;

    // --- HIGH QUALITY RENDER PASS (Spatial + Temporal Smoothing) ---
#pragma omp parallel for default(none) shared(obs, src, sm, bmp, w, h, invMaxV, invC, lut, e) schedule(static) \
    num_threads(e->numThreads)
    for (int y = 0; y < h; y++) {
        bool y_edge = (y == 0 || y == h - 1);
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;
            if (obs[idx]) { sm[idx] = 0; bmp[idx] = 0xFFFFFFFF; continue; }

            float sum = 0;
            int count = 0;

            if (!y_edge && x > 0 && x < w - 1) {
                // Fast Path Interior
                int t = idx - w; int b = idx + w;
                sum = src[t-1]+src[t]+src[t+1]+src[idx-1]+src[idx]+src[idx+1]+src[b-1]+src[b]+src[b+1];
                count = (1-obs[t-1])+(1-obs[t])+(1-obs[t+1])+(1-obs[idx-1])+(1-obs[idx])+(1-obs[idx+1])+(1-obs[b-1])+(1-obs[b])+(1-obs[b+1]);
            } else {
                // Slow Path Edges
                for (int dy = -1; dy <= 1; dy++) {
                    int ny = y + dy;
                    if (ny >= 0 && ny < h) {
                        for (int dx = -1; dx <= 1; dx++) {
                            int nx = x + dx;
                            if (nx >= 0 && nx < w) {
                                int nidx = ny * w + nx;
                                sum += src[nidx];
                                count += (1 - obs[nidx]);
                            }
                        }
                    }
                }
            }

            float rv = sm[idx] * 0.7f + (sum * invC[count]) * 0.3f;
            sm[idx] = rv;
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
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDragCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->smoothedCd;
}
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getInstantDragCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->dragCoefficient;
}
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getLiftForceNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->liftForceNewtons; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getLiftCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->smoothedCl;
}
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getInstantLiftCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->liftCoefficient;
}
extern "C" JNIEXPORT jlong JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getTotalStepsNative(JNIEnv *env, jobject thiz, jlong ptr) { return (jlong)reinterpret_cast<LBMEngine*>(ptr)->totalSteps; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDensityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->rhoAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getViscosityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->nuAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDXNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->dx; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getHorizontalSpanNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->horizontalSpan; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getFrontalAreaNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->frontalArea; }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setVisualizationModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint m) { reinterpret_cast<LBMEngine*>(ptr)->vizMode = static_cast<LBMEngine::VisualizationMode>(m); }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setBoundaryModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint mode) {
    reinterpret_cast<LBMEngine*>(ptr)->boundaryMode = static_cast<LBMEngine::BoundaryMode>(mode);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getMaxCoresNative(JNIEnv *env, jobject thiz) {
    return omp_get_num_procs();
}

extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setNumThreadsNative(JNIEnv *env, jobject thiz, jlong ptr, jint n) {
    if (n > 0) reinterpret_cast<LBMEngine*>(ptr)->numThreads = n;
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getActiveCoresNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->activeCores;
}
