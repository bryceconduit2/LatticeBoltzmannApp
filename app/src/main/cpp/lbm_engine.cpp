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
 * Converts a 0.0 to 1.0 value into an ARGB color.
 * Blue (0.0) -> Cyan -> Green -> Yellow -> Red (1.0)
 */
inline uint32_t heatMapColor(float value) {
    value = fmaxf(0.0f, fminf(1.0f, value));
    float r, g, b;
    if (value < 0.25f) { // Blue to Cyan
        r = 0.0f; g = 4.0f * value; b = 1.0f;
    } else if (value < 0.5f) { // Cyan to Green
        r = 0.0f; g = 1.0f; b = 1.0f - 4.0f * (value - 0.25f);
    } else if (value < 0.75f) { // Green to Yellow
        r = 4.0f * (value - 0.5f); g = 1.0f; b = 0.0f;
    } else { // Yellow to Red
        r = 1.0f; g = 1.0f - 4.0f * (value - 0.75f); b = 0.0f;
    }
    // Android Bitmap ARGB_8888 is stored as 0xAABBGGRR in little-endian memory
    return 0xFF000000 | (static_cast<uint32_t>(b * 255.0f) << 16) | (static_cast<uint32_t>(g * 255.0f) << 8) | static_cast<uint32_t>(r * 255.0f);
}

struct LBMEngine {
    enum VisualizationMode { VELOCITY = 0, PRESSURE = 1, TOTAL_PRESSURE = 2 };

    int width, height;
    float uInlet = 0.06f;       // Current physical inlet velocity in lattice units
    float uInletTarget = 0.06f; // User-requested velocity (smoothed toward this)
    float omega = 1.0f / 0.55f; // Relaxation time (calculated from viscosity)
    const float cs2 = 1.0f / 3.0f; // Speed of sound squared
    const float SmagorinskyConstant = 0.16f; // For turbulence modeling

    // Physical Constants
    const float dx = 0.0025f;    // Grid spacing (m)
    const float dt = 0.000005f;  // Time step (s)
    float rhoAir = 1.225f;       // Physical density (kg/m^3)
    float nuAir = 1.5e-5f;       // Kinematic viscosity (m^2/s)

    // Telemetry results for the UI
    float dragForceNewtons = 0.0f;
    float dragCoefficient = 0.0f;
    float frontalArea = 0.0f; // Projected height of all obstacles

    VisualizationMode vizMode = VELOCITY;

    // Lattice Grids (Structure of Arrays for SIMD performance)
    std::vector<float> f[9];    // Current distribution functions
    std::vector<float> fNew[9]; // Buffer for the next time step
    std::vector<uint8_t> obstacles; // 1 if cell is solid, 0 if fluid
    std::vector<float> velocityMag; // Pre-calculated magnitude for rendering
    std::vector<float> visualizationSource; // Raw map data (velocity or pressure)
    std::vector<float> smoothedVelocityMag; // Temporally smoothed data for the heatmap

    uint32_t colorLUT[256]; // Precomputed colors for fast rendering

    // D2Q9 direction constants
    // Change these from 'const float' to 'static constexpr float'
    static constexpr float cxs[9] = {0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 1.0f, -1.0f, -1.0f, 1.0f};
    static constexpr float cys[9] = {0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f};
    static constexpr float weights[9] = {4.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/36, 1.f/36, 1.f/36, 1.f/36};
    static constexpr int opposite[9] = {0, 3, 4, 1, 2, 7, 8, 5, 6};
    static constexpr float cxx[9] = {0, 1, 0, 1, 0, 1, 1, 1, 1};
    static constexpr float cyy[9] = {0, 0, 1, 0, 1, 1, 1, 1, 1};
    static constexpr float cxy[9] = {0, 0, 0, 0, 0, 1, -1, 1, -1};

    LBMEngine(int w, int h) : width(w), height(h) {
        // Precompute color mapping to avoid expensive math in the render loop
        for (int i = 0; i < 256; i++) colorLUT[i] = heatMapColor(i / 255.0f);

        // Optimization: limit OpenMP to physical high-performance cores
        int numProcs = omp_get_num_procs();
        omp_set_num_threads(numProcs >= 8 ? 4 : numProcs);

        int size = width * height;
        for (int i = 0; i < 9; i++) {
            f[i].resize(size, 0.0f);
            fNew[i].resize(size, 0.0f);
        }
        obstacles.resize(size, 0);
        velocityMag.resize(size, 0.0f);
        visualizationSource.resize(size, 0.0f);
        smoothedVelocityMag.resize(size, 0.0f);

        initFluid(uInlet);
    }

    /**
     * Initializes the fluid field to an equilibrium state at a specific velocity.
     */
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
        std::fill(velocityMag.begin(), velocityMag.end(), 0.0f);
        std::fill(visualizationSource.begin(), visualizationSource.end(), 0.0f);
        std::fill(smoothedVelocityMag.begin(), smoothedVelocityMag.end(), 0.0f);
        initFluid(uInletTarget);
    }

    /**
     * Executes a single time step of the LBM algorithm.
     * Performs Streaming, Macroscopic updates, Collision, and Boundary conditions.
     */
    float step() {
        // Smoothly transition velocity to prevent numerical shocks
        uInlet = uInlet * 0.99f + uInletTarget * 0.01f;

        // Dynamic relaxation time based on physical viscosity
        float nuLB = nuAir * dt / (dx * dx);
        float tau_0 = 3.0f * nuLB + 0.5f;
        float inv2cs2 = 1.5f;
        float inv2cs4 = 4.5f;
        float regFactor = 0.98f * inv2cs4;
        float smagCoeff = 18.0f * SmagorinskyConstant * SmagorinskyConstant;

        float localDragX = 0.0f; // Track horizontal momentum exchange (drag)
        int stepMinY = height, stepMaxY = 0; // For projected frontal area calculation

        static const int CX[9] = { 0, 1, 0, -1, 0, 1, -1, -1, 1 };
        static const int CY[9] = { 0, 0, 1, 0, -1, 1, 1, -1, -1 };

        // Pointers for Structure of Arrays (SoA) access
        float* f_ptr[9];
        float* fn_ptr[9];
        for (int i = 0; i < 9; i++) { f_ptr[i] = f[i].data(); fn_ptr[i] = fNew[i].data(); }
        const uint8_t* __restrict obs_ptr = obstacles.data();
        float* __restrict vel_ptr = velocityMag.data();
        float* __restrict viz_ptr = visualizationSource.data();

        VisualizationMode mode = vizMode;
        float pressScale = (uInlet > 0.001f) ? (1.0f / (uInlet * uInlet)) : 100.0f;

        // --- STAGE 1: Inlet Boundary (x = 0) ---
        // Handles specialized velocity boundary conditions at the entrance
#pragma omp parallel for schedule(static) default(none) \
    shared(f_ptr, fn_ptr, obs_ptr, vel_ptr, viz_ptr, width, height, uInlet, weights, cxs, opposite, CX, CY, mode, pressScale)
        for (int y = 0; y < height; y++) {
            int idx = y * width;
            if (obs_ptr[idx]) {
                // Bounce-back for solids at the boundary
                for (int i = 0; i < 9; i++) {
                    int nx = CX[i]; // Look in the direction the bounced particle is going
                    int ny = y + CY[i];

                    if (nx >= width) nx = 0; // Standard bounds check
                    if (ny < 0) ny = height - 1; else if (ny >= height) ny = 0;

                    fn_ptr[i][idx] = f_ptr[opposite[i]][ny * width + nx];
                }
                vel_ptr[idx] = 0.0f; viz_ptr[idx] = 0.0f;
            } else {
                // Prescribed velocity at the inlet
                float ux = uInlet; vel_ptr[idx] = ux;
                viz_ptr[idx] = (mode == VELOCITY) ? ux : (mode == PRESSURE ? 0.5f : 0.5f + (0.5f * ux * ux) * pressScale);
                float usqr = 1.5f * ux * ux;
                for (int i = 0; i < 9; i++) {
                    float cu = 3.0f * cxs[i] * ux;
                    fn_ptr[i][idx] = weights[i] * (1.0f + cu + 0.5f * cu * cu - usqr);
                }
            }
        }

        // --- STAGE 2: Main Simulation (Collision & Streaming) ---
#pragma omp parallel for collapse(2) schedule(static) default(none) \
    shared(f_ptr, fn_ptr, obs_ptr, vel_ptr, viz_ptr, width, height, cs2, smagCoeff, inv2cs2, \
           tau_0, regFactor, weights, cxs, cys, opposite, cxx, cyy, cxy, CX, CY, mode, pressScale) \
    reduction(+:localDragX) reduction(min:stepMinY) reduction(max:stepMaxY)
        for (int y = 0; y < height; y++) {
            for (int x = 1; x < width; x++) {
                int idx = y * width + x;
                float local_f[9];

                if (obs_ptr[idx]) {
                    // --- Solid Collision (Bounce-Back) ---
                    for (int i = 0; i < 9; i++) {
                        int nx = x - CX[i]; int ny = y - CY[i];
                        if (nx >= width) nx = 0; if (ny < 0) ny = height - 1; else if (ny >= height) ny = 0;
                        float fi = f_ptr[i][ny * width + nx]; local_f[i] = fi;

                        // Momentum Exchange Method for Drag: Force = 2 * (f_i - equilibrium_base)
                        if (!obs_ptr[ny * width + nx]) localDragX += 2.0f * (fi - weights[i]) * cxs[i];
                    }
                    for (int i = 0; i < 9; i++) fn_ptr[i][idx] = local_f[opposite[i]];
                    vel_ptr[idx] = 0.0f; viz_ptr[idx] = 0.0f;
                    if (y < stepMinY) stepMinY = y; if (y > stepMaxY) stepMaxY = y;
                } else {
                    // --- Fluid Collision (BGK + Smagorinsky) ---
                    // 1. PULL STREAMING (Collect values from neighbors)
                    for (int i = 0; i < 9; i++) {
                        int nx = x - CX[i]; int ny = y - CY[i];
                        if (nx >= width) nx = 0; if (ny < 0) ny = height - 1; else if (ny >= height) ny = 0;
                        local_f[i] = f_ptr[i][ny * width + nx];
                    }

                    // 2. MACROSCOPIC MOMENTS (Density and Velocity)
                    float rho = local_f[0]+local_f[1]+local_f[2]+local_f[3]+local_f[4]+local_f[5]+local_f[6]+local_f[7]+local_f[8];
                    rho = fmaxf(0.1f, rho); float invRho = 1.0f / rho;
                    float ux = (local_f[1] - local_f[3] + local_f[5] - local_f[6] - local_f[7] + local_f[8]) * invRho;
                    float uy = (local_f[2] - local_f[4] + local_f[5] + local_f[6] - local_f[7] - local_f[8]) * invRho;

                    float vMag2 = ux * ux + uy * uy; float vMag = sqrtf(vMag2);
                    if (vMag > 0.5f) { float s = 0.5f / vMag; ux *= s; uy *= s; vMag = 0.5f; vMag2 = 0.25f; }
                    vel_ptr[idx] = vMag;

                    // 3. MAP DATA for the UI Heatmap
                    if (mode == VELOCITY) viz_ptr[idx] = vMag;
                    else if (mode == PRESSURE) viz_ptr[idx] = 0.5f + ((rho - 1.0f) * cs2) * pressScale;
                    else viz_ptr[idx] = 0.5f + ((rho - 1.0f) * cs2 + 0.5f * rho * vMag2) * pressScale;

                    // 4. EQUILIBRIUM & STRESS TENSOR
                    float usqr = 1.5f * vMag2;
                    float pixx = 0, pixy = 0, piyy = 0;
                    float fi_eq[9];
                    for (int i = 0; i < 9; i++) {
                        float cu = cxs[i] * ux + cys[i] * uy;
                        fi_eq[i] = rho * weights[i] * (1.0f + 3.0f * cu + 4.5f * cu * cu - usqr);
                        float fi_neq = local_f[i] - fi_eq[i];
                        pixx += fi_neq * cxx[i]; pixy += fi_neq * cxy[i]; piyy += fi_neq * cyy[i];
                    }

                    // 5. SMAGORINSKY LES (Eddy Viscosity)
                    float S = sqrtf(pixx * pixx + 2.0f * pixy * pixy + piyy * piyy) * invRho * inv2cs2;
                    float tau_eff = tau_0 + 0.5f * (sqrtf(tau_0 * tau_0 + smagCoeff * S) - tau_0);
                    float one_minus_invTau = 1.0f - (1.0f / fmaxf(tau_eff, 0.501f));

                    // 6. COLLISION (Write out to fNew)
                    for (int i = 0; i < 9; i++) {
                        float Qpix = (cxx[i] - cs2) * pixx + 2.0f * cxy[i] * pixy + (cyy[i] - cs2) * piyy;
                        fn_ptr[i][idx] = fi_eq[i] + one_minus_invTau * weights[i] * regFactor * Qpix;
                    }
                }
            }
        }

        // Track the projected height of all obstacles to compute Cd correctly
        if (stepMaxY >= stepMinY) frontalArea = static_cast<float>(stepMaxY - stepMinY + 1) * dx; else frontalArea = 0.0f;

        // SWAP grids for the next time step
        for (int i = 0; i < 9; i++) f[i].swap(fNew[i]);

        return localDragX;
    }
};

extern "C" JNIEXPORT jlong JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_initEngine(JNIEnv *env, jobject thiz, jint width, jint height) {
    return (jlong)(uintptr_t)new LBMEngine(width, height);
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_destroyEngine(JNIEnv *env, jobject thiz, jlong ptr) {
    delete reinterpret_cast<LBMEngine*>(ptr);
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_addObstacleNative(JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint radius) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); int r2 = radius * radius;
    for (int y = std::max(0, cy - radius); y <= std::min(e->height - 1, cy + radius); y++)
        for (int x = std::max(0, cx - radius); x <= std::min(e->width - 1, cx + radius); x++)
            if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2) e->obstacles[y * e->width + x] = 1;
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_addBoxObstacleNative(JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint size) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); int h = size / 2;
    for (int y = std::max(0, cy - h); y <= std::min(e->height - 1, cy + h); y++)
        for (int x = std::max(0, cx - h); x <= std::min(e->width - 1, cx + h); x++)
            e->obstacles[y * e->width + x] = 1;
}
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setDensityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat d) { reinterpret_cast<LBMEngine*>(ptr)->rhoAir = d; }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setViscosityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat v) { reinterpret_cast<LBMEngine*>(ptr)->nuAir = v; }

/**
 * Executes a number of simulation steps and renders the current state to a Bitmap.
 * Also performs high-precision drag force averaging.
 */
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_stepAndRenderNative(JNIEnv *env, jobject thiz, jlong ptr, jobject bitmap, jint steps) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); double dragAcc = 0.0;

    // 1. Run simulation steps and accumulate momentum exchange
    for (int i = 0; i < steps; i++) dragAcc += (double)e->step();

    if (steps > 0) {
        float avgF = (float)(dragAcc / steps);
        // Convert averaged Lattice force to physical Newtons
        //float latticeToNewton = e->rhoAir * (e->dx * e->dx) * (e->dx / (e->dt * e->dt)) * (1.0f / 3.0f);
        //e->dragForceNewtons = fabsf(avgF) * latticeToNewton;
        //physics bodge to get correct asnwer CHECK!!
        float L_z = 1.0f; // Actual physical depth of your object in meters
        float latticeToNewton = e->rhoAir * L_z * (e->dx * e->dx* e->dx) / (e->dt* e->dt)* (1.0f / 18.0f);
        e->dragForceNewtons = fabsf(avgF)  * latticeToNewton;

        float uL = e->uInlet; float areaL = e->frontalArea / e->dx;
        // Compute Non-dimensional Cd = Force / (Dynamic Pressure * Area)
        e->dragCoefficient = (uL > 0.001f && areaL > 0.5f) ? fabsf(avgF) / (0.5f * (uL * uL) * areaL) : 0.0f;

        static int logCounter = 0;
        if (++logCounter % 60 == 0) {
            LOGI("Physics: Steps=%d, SumF=%.4f, avgF=%.4f, Newtons=%.1f", steps, (float)dragAcc, avgF, e->dragForceNewtons);
        }
    }

    // 2. Lock bitmap and render the heatmap
    void* pixels; if (AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) return;
    auto* bmp = static_cast<uint32_t*>(pixels);
    int w = e->width; int h = e->height; float invMaxV = 255.0f / (e->uInlet * 1.8f);
    static const float invC[10] = { 0, 1.0f, 0.5f, 0.3333333f, 0.25f, 0.2f, 0.1666667f, 0.1428571f, 0.125f, 0.1111111f };
    const uint8_t* obs = e->obstacles.data(); const float* src = e->visualizationSource.data();
    float* sm = e->smoothedVelocityMag.data(); uint32_t* lut = e->colorLUT;

#pragma omp parallel for default(none) shared(obs, src, sm, bmp, w, h, invMaxV, invC, lut, e) schedule(static)
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;
            if (obs[idx]) { sm[idx] = 0; bmp[idx] = 0xFFFFFFFF; continue; } // White for solid

            // Spatial Smoothing (3x3 average)
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

            // Temporal Smoothing (Damping)
            float rv = sm[idx] * 0.7f + (sum * invC[count]) * 0.3f; sm[idx] = rv;

            // Scale and Map to Color LUT
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
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDensityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->rhoAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getViscosityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->nuAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_getDXNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->dx; }
extern "C" JNIEXPORT void JNICALL Java_com_example_latticeboltzmann_NativeLBMEngine_setVisualizationModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint m) { reinterpret_cast<LBMEngine*>(ptr)->vizMode = static_cast<LBMEngine::VisualizationMode>(m); }
