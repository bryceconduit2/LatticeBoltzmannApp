/**
 * Fluid Sandbox Physics Engine
 * Core Implementation: Lattice Boltzmann Method (LBM)
 *
 * Technical Specifications:
 * - Lattice Model: D2Q9 (2 Dimensions, 9 Velocity Vectors)
 * - Collision Operator: Regularized BGK (Bhatnagar-Gross-Krook)
 * - Turbulence Model: Smagorinsky Subgrid-Scale (LES)
 * - Boundary Scheme: Bouzidi–Firdaouss–Lallemand (BFL) for curved/interpolated walls
 * - Parallelism: OpenMP (Shared memory multi-threading)
 */

#include <jni.h>
#include <android/bitmap.h>
#include <android/log.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <omp.h>
#include <chrono>

// Debugging macros - Disabled in Release builds for performance
#define LOG_TAG "LBM_ENGINE"
#ifdef NDEBUG
#define LOGI(...) ((void)0)
#else
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#endif

/**
 * LBM weights for the D2Q9 model.
 * Sum to 1.0; used to calculate equilibrium distributions.
 */
const float W0 = 4.0f/9.0f;
const float W1 = 1.0f/9.0f;
const float W2 = 1.0f/36.0f;

/**
 * Fast Heatmap Color Mapper.
 * Maps normalized fluid property (0.0 to 1.0) to a high-contrast temperature scale.
 * Colors: Blue (Slow) -> Cyan -> Green -> Yellow -> Red (Fast)
 */
inline uint32_t heatMapColor(float value) {
    value = fmaxf(0.0f, fminf(1.0f, value));
    float r, g, b;
    if (value < 0.25f) { r = 0.0f; g = 4.0f * value; b = 1.0f; }
    else if (value < 0.5f) { r = 0.0f; g = 1.0f; b = 1.0f - 4.0f * (value - 0.25f); }
    else if (value < 0.75f) { r = 4.0f * (value - 0.5f); g = 1.0f; b = 0.0f; }
    else { r = 1.0f; g = 1.0f - 4.0f * (value - 0.75f); b = 0.0f; }
    // Android Bitmap ARGB_8888 (Little Endian: 0xAABBGGRR)
    return 0xFF000000 | (static_cast<uint32_t>(b * 255.0f) << 16) | (static_cast<uint32_t>(g * 255.0f) << 8) | static_cast<uint32_t>(r * 255.0f);
}

/**
 * Data structure to return integrated aerodynamic forces.
 */
struct ForceResult {
    float drag;
    float lift;
};

/**
 * MAIN ENGINE CLASS
 * Encapsulates the entire Lattice Boltzmann grid and solver state.
 */
struct LBMEngine {
    enum VisualizationMode { VELOCITY = 0, PRESSURE = 1, TOTAL_PRESSURE = 2 };
    enum BoundaryMode { PERIODIC = 0, NO_SLIP = 1, FREE_SLIP = 2 };

    int width, height;
    float uInlet = 0.06f;       // Current inlet velocity in lattice units
    float uInletTarget = 0.06f; // Target velocity for smoothing
    float omega = 1.0f / 0.55f; // Base relaxation frequency
    const float cs2 = 1.0f / 3.0f; // Speed of sound squared
    const float SmagorinskyConstant = 0.16f; // Subgrid turbulence parameter

    // Physical calibration constants
    const float dx = 0.0025f;     // Grid spacing (meters)
    float dt = 0.000005f;         // Time step (seconds)
    float rhoAir = 1.225f;        // Density (kg/m^3)
    float nuAir = 1.5e-5f;        // Kinematic viscosity (m^2/s)

    // Telemetry readouts
    float dragForceNewtons = 0.0f;
    float dragCoefficient = 0.0f;
    float liftForceNewtons = 0.0f;
    float liftCoefficient = 0.0f;
    float smoothedCd = 0.0f;
    float smoothedCl = 0.0f;
    bool localRefinementEnabled = false;
    int numThreads = 4;
    int activeCores = 0;
    float frontalArea = 0.0f;
    float horizontalSpan = 0.0f;
    unsigned long long totalSteps = 0;

    VisualizationMode vizMode = VELOCITY;
    BoundaryMode boundaryMode = PERIODIC;

    // Distribution functions (Stored as Structure-of-Arrays for SIMD optimization)
    std::vector<float> f[9];
    std::vector<float> fNew[9];
    std::vector<uint8_t> obstacles;
    std::vector<float> linkQ[9]; // BFL: fractional distance to actual curved wall

    // --- LOCAL REFINEMENT (2:1 NESTED GRID) ---
    std::vector<float> fine_f[9];
    std::vector<float> fine_fNew[9];
    int roiX1, roiY1, roiX2, roiY2; // Bounds of the fine grid in Coarse units
    int fine_w, fine_h;

    std::vector<float> velocityMag;
    std::vector<float> visualizationSource;
    std::vector<float> smoothedVelocityMag;

    uint32_t colorLUT[256];

    // D2Q9 Lattice Vectors
    const float cxs[9] = {0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 1.0f, -1.0f, -1.0f, 1.0f};
    const float cys[9] = {0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f};
    const float weights[9] = {4.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/36, 1.f/36, 1.f/36, 1.f/36};
    const int opposite[9] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

    // Precomputed components for the stress tensor calculation
    const float cxx[9] = {0, 1, 0, 1, 0, 1, 1, 1, 1};
    const float cyy[9] = {0, 0, 1, 0, 1, 1, 1, 1, 1};
    const float cxy[9] = {0, 0, 0, 0, 0, 1, -1, 1, -1};

    LBMEngine(int w, int h) : width(w), height(h), roiX1(0), roiY1(0), roiX2(0), roiY2(0), fine_w(0), fine_h(0) {
        for (int i = 0; i < 256; i++) colorLUT[i] = heatMapColor(i / 255.0f);
        int numProcs = omp_get_num_procs();
        numThreads = (numProcs >= 8 ? 4 : numProcs);
        int size = width * height;
        for (int i = 0; i < 9; i++) {
            f[i].resize(size, 0.0f);
            fNew[i].resize(size, 0.0f);
            linkQ[i].resize(size, 0.5f);
        }
        obstacles.resize(size, 0);
        velocityMag.resize(size, 0.0f);
        visualizationSource.resize(size, 0.0f);
        smoothedVelocityMag.resize(size, 0.0f);
        initFluid(uInlet);
    }

    /**
     * Initializes the grid with a uniform flow field.
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
        for (int i = 0; i < 9; i++) std::fill(linkQ[i].begin(), linkQ[i].end(), 0.5f);
        std::fill(velocityMag.begin(), velocityMag.end(), 0.0f);
        std::fill(visualizationSource.begin(), visualizationSource.end(), 0.0f);
        std::fill(smoothedVelocityMag.begin(), smoothedVelocityMag.end(), 0.0f);
        totalSteps = 0;
        smoothedCd = 0.0f;
        smoothedCl = 0.0f;
        initFluid(uInletTarget);
    }

    /**
     * CORE SIMULATION KERNEL.
     * Executes a single Time Step. Fused logic significantly improves cache performance.
     */
    ForceResult step() {
        totalSteps++;
        // Smoothly adjust inlet velocity to prevent numerical shocks
        uInlet = uInlet * 0.99f + uInletTarget * 0.01f;

        // Calculate lattice-space viscosity and relaxation time
        const float nuLB = nuAir * dt / (dx * dx);
        const float tau_0 = 3.0f * nuLB + 0.5f;

        const float inv2cs2 = 1.5f;
        const float inv2cs4 = 4.5f;
        const float regFactor = 0.98f * inv2cs4;
        const float smagCoeff = 18.0f * SmagorinskyConstant * SmagorinskyConstant;

        double totalForceX = 0.0; double totalForceY = 0.0;
        int stepMinY = height, stepMaxY = 0;
        int stepMinX = width, stepMaxX = 0;

        // --- ROI DETECTION (For Local Refinement) ---
        if (localRefinementEnabled) {
            // Find bounding box of all obstacles
            for (int idx = 0; idx < width * height; idx++) {
                if (obstacles[idx]) {
                    int x = idx % width; int y = idx / width;
                    if (x < stepMinX) stepMinX = x; if (x > stepMaxX) stepMaxX = x;
                    if (y < stepMinY) stepMinY = y; if (y > stepMaxY) stepMaxY = y;
                }
            }
            // Add a safety margin (e.g., 15 cells)
            roiX1 = std::max(0, stepMinX - 15);
            roiY1 = std::max(0, stepMinY - 15);
            roiX2 = std::min(width - 1, stepMaxX + 15);
            roiY2 = std::min(height - 1, stepMaxY + 15);

            fine_w = (roiX2 - roiX1 + 1) * 2;
            fine_h = (roiY2 - roiY1 + 1) * 2;

            // Resize fine grid if needed
            if (fine_f[0].size() != (size_t)(fine_w * fine_h)) {
                for (int i = 0; i < 9; i++) {
                    fine_f[i].assign(fine_w * fine_h, weights[i]);
                    fine_fNew[i].resize(fine_w * fine_h);
                }
            }
        }

        static const int CX[9] = { 0, 1, 0, -1, 0, 1, -1, -1, 1 };
        static const int CY[9] = { 0, 0, 1, 0, -1, 1, 1, -1, -1 };

        float* f_ptr[9]; float* fn_ptr[9];
        for (int i = 0; i < 9; i++) {
            f_ptr[i] = f[i].data();
            fn_ptr[i] = fNew[i].data();
        }
        const uint8_t* __restrict obs_ptr = obstacles.data();
        const float* __restrict lq_ptr[9];
        for (int i = 0; i < 9; i++) lq_ptr[i] = linkQ[i].data();

#pragma omp parallel for schedule(static) default(none) \
    shared(f_ptr, fn_ptr, obs_ptr, lq_ptr, velocityMag, visualizationSource, width, height, cs2, smagCoeff, inv2cs2, tau_0, regFactor, weights, CX, CY, opposite, vizMode, uInlet, numThreads, boundaryMode, W0, W1, W2) \
    reduction(+:totalForceX, totalForceY) reduction(min:stepMinY, stepMinX) reduction(max:stepMaxY, stepMaxX) \
    num_threads(numThreads)
        for (int y = 0; y < height; y++) {
            if (omp_get_thread_num() == 0) activeCores = omp_get_num_threads();
            const bool y_edge = (y == 0 || y == height - 1);

            for (int x = 0; x < width; x++) {
                const int idx = y * width + x;
                if (obs_ptr[idx]) {
                    // Update bounding box of obstacles for area-based aerodynamics
                    if (y < stepMinY) stepMinY = y; if (y > stepMaxY) stepMaxY = y;
                    if (x < stepMinX) stepMinX = x; if (x > stepMaxX) stepMaxX = x;
                    continue;
                }

                // --- 1. PULL STREAMING & BOUNDARY COLLISION ---
                float local_f[9];

                if (x == 0) {
                    // Inlet Boundary: Constant velocity equilibrium
                    const float u = uInlet;
                    const float term1 = 1.0f - 1.5f * u * u;
                    const float u3 = 3.0f * u;
                    const float u45 = 4.5f * u * u;
                    local_f[0] = W0 * term1;
                    local_f[1] = W1 * (term1 + u3 + u45);
                    local_f[2] = W1 * term1;
                    local_f[3] = W1 * (term1 - u3 + u45);
                    local_f[4] = W1 * term1;
                    local_f[5] = W2 * (term1 + u3 + u45);
                    local_f[6] = W2 * (term1 - u3 + u45);
                    local_f[7] = W2 * (term1 - u3 + u45);
                    local_f[8] = W2 * (term1 + u3 + u45);
                } else {
                    for (int i = 0; i < 9; i++) {
                        int nx = x - CX[i];
                        int ny = y - CY[i];

                        // Top/Bottom walls or Periodic wrap
                        if (nx < 0) nx = width - 1; else if (nx >= width) nx = 0;
                        if (ny < 0 || ny >= height) {
                            if (boundaryMode == PERIODIC) {
                                if (ny < 0) ny = height - 1; else ny = 0;
                            } else {
                                local_f[i] = f_ptr[opposite[i]][idx];
                                continue;
                            }
                        }

                        const int srcIdx = ny * width + nx;
                        if (obs_ptr[srcIdx]) {
                            // INTERPOLATED BOUNDARY CONDITION (BFL Scheme)
                            // Calculates exact reflection based on obstacle surface distance 'q'
                            const int dir_to_wall = opposite[i];
                            const float q = lq_ptr[dir_to_wall][idx];
                            const float fi_out = f_ptr[dir_to_wall][idx];
                            float fi_in;

                            if (q < 0.5f) {
                                int nx2 = x + CX[i]; int ny2 = y + CY[i];
                                if (nx2 < 0) nx2 = width-1; else if (nx2 >= width) nx2 = 0;
                                if (ny2 < 0) ny2 = height-1; else if (ny2 >= height) ny2 = 0;
                                fi_in = 2.0f * q * fi_out + (1.0f - 2.0f * q) * f_ptr[dir_to_wall][ny2 * width + nx2];
                            } else {
                                float inv2q = 1.0f / (2.0f * q);
                                fi_in = inv2q * fi_out + (1.0f - inv2q) * f_ptr[i][idx];
                            }
                            local_f[i] = fi_in;

                            // Momentum Exchange: Integrate forces (Drag and Lift)
                            totalForceX += (double)(fi_out + fi_in) * (double)CX[dir_to_wall];
                            totalForceY += (double)(fi_out + fi_in) * (double)CY[dir_to_wall];
                        } else {
                            local_f[i] = f_ptr[i][srcIdx]; // Regular fluid streaming
                        }
                    }
                }

                // --- 2. MACROSCOPIC MOMENTS ---
                // Calculate Density (Rho) and Velocity (u)
                const float rho = local_f[0] + local_f[1] + local_f[2] + local_f[3] + local_f[4] + local_f[5] + local_f[6] + local_f[7] + local_f[8];
                const float invRho = 1.0f / fmaxf(0.1f, rho);
                float ux = (local_f[1] - local_f[3] + local_f[5] - local_f[6] - local_f[7] + local_f[8]) * invRho;
                float uy = (local_f[2] - local_f[4] + local_f[5] + local_f[6] - local_f[7] - local_f[8]) * invRho;

                float vMag2 = ux * ux + uy * uy; float vMag = sqrtf(vMag2);
                if (vMag > 0.45f) { float s = 0.45f / vMag; ux *= s; uy *= s; vMag = 0.45f; vMag2 = 0.2025f; }
                velocityMag[idx] = vMag;

                // --- 3. COLLISION (Regularized BGK + Smagorinsky) ---
                // Regularization filters out non-physical modes for higher stability
                const float one_minus_15u2 = 1.0f - 1.5f * vMag2;
                const float row_w_9 = rho * W1;
                const float row_w_36 = rho * W2;
                const float ux3 = 3.0f * ux; const float uy3 = 3.0f * uy;
                const float ux9 = 4.5f * ux * ux; const float uy9 = 4.5f * uy * uy;
                const float uxy9 = 9.0f * ux * uy;

                float feq[9]; // Equilibrium Distribution
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

                // Compute local Stress Tensor for the Smagorinsky model
                float pixx = (local_f[1]-feq[1]) + (local_f[3]-feq[3]) + (local_f[5]-feq[5]) + (local_f[6]-feq[6]) + (local_f[7]-feq[7]) + (local_f[8]-feq[8]);
                float piyy = (local_f[2]-feq[2]) + (local_f[4]-feq[4]) + (local_f[5]-feq[5]) + (local_f[6]-feq[6]) + (local_f[7]-feq[7]) + (local_f[8]-feq[8]);
                float pixy = (local_f[5]-feq[5]) - (local_f[6]-feq[6]) + (local_f[7]-feq[7]) - (local_f[8]-feq[8]);

                // Smagorinsky LES: Adjusts viscosity based on local turbulence intensity
                const float S = sqrtf(pixx * pixx + 2.0f * pixy * pixy + piyy * piyy) * invRho * inv2cs2;
                const float tau_eff = tau_0 + 0.5f * (sqrtf(tau_0 * tau_0 + smagCoeff * S) - tau_0);
                const float one_minus_invTau = 1.0f - (1.0f / fmaxf(tau_eff, 0.501f));
                const float common_reg = one_minus_invTau * regFactor;

                const float reg_pixx = common_reg * (pixx * (1.0f - cs2) + piyy * (0.0f - cs2));
                const float reg_piyy = common_reg * (pixx * (0.0f - cs2) + piyy * (1.0f - cs2));
                const float reg_pixy = common_reg * 2.0f * pixy;

                // FINAL COLLISION STEP: Store result in fNew
                fn_ptr[0][idx] = feq[0] + W0 * common_reg * (-cs2 * (pixx + piyy));
                fn_ptr[1][idx] = feq[1] + W1 * reg_pixx;
                fn_ptr[2][idx] = feq[2] + W1 * reg_piyy;
                fn_ptr[3][idx] = feq[3] + W1 * reg_pixx;
                fn_ptr[4][idx] = feq[4] + W1 * reg_piyy;
                fn_ptr[5][idx] = feq[5] + W2 * (reg_pixx + reg_pixy + reg_piyy);
                fn_ptr[6][idx] = feq[6] + W2 * (reg_pixx - reg_pixy + reg_piyy);
                fn_ptr[7][idx] = feq[7] + W2 * (reg_pixx + reg_pixy + reg_piyy);
                fn_ptr[8][idx] = feq[8] + W2 * (reg_pixx - reg_pixy + reg_piyy);

                // Numerical Safety: Check for NaN divergence
                if (std::isnan(fn_ptr[0][idx])) { for(int i=0; i<9; i++) fn_ptr[i][idx] = weights[i]; }

                // --- DATA PREPARATION FOR VISUALIZATION ---
                const float ps = (rho - 1.0f) * cs2;
                const float pressScale = (uInlet > 0.001f) ? (1.0f / (uInlet * uInlet)) : 100.0f;
                if (vizMode == VELOCITY) visualizationSource[idx] = vMag;
                else if (vizMode == PRESSURE) visualizationSource[idx] = 0.5f + ps * pressScale;
                else visualizationSource[idx] = 0.5f + (ps + 0.5f * rho * vMag2) * pressScale;
            }
        }

        // Finalize aerodynamic geometric stats
        if (stepMaxY >= stepMinY) frontalArea = static_cast<float>(stepMaxY - stepMinY + 1) * dx; else frontalArea = 0.0f;
        if (stepMaxX >= stepMinX) horizontalSpan = static_cast<float>(stepMaxX - stepMinX + 1) * dx; else horizontalSpan = 0.0f;

        // --- LOCAL REFINEMENT SUB-STEPPING ---
        // (Simplified Refined Solver for ROI)
        if (localRefinementEnabled && (stepMaxX >= stepMinX)) {
            // For now, we perform an additional 'High Accuracy' pass on the coarse nodes in the ROI
            // This increases the effective convergence rate for boundary layers.
            #pragma omp parallel for schedule(static) shared(f_ptr, fn_ptr, obs_ptr, weights, tau_0, regFactor) num_threads(numThreads)
            for (int y = roiY1; y <= roiY2; y++) {
                for (int x = roiX1; x <= roiX2; x++) {
                    int idx = y * width + x;
                    if (obs_ptr[idx]) continue;

                    // Sample current macroscopic moments for the ROI node
                    float rho_roi = 0;
                    for(int i=0; i<9; i++) rho_roi += f_ptr[i][idx];
                    float invRho_roi = 1.0f / fmaxf(0.1f, rho_roi);
                    float ux_roi = (f_ptr[1][idx] - f_ptr[3][idx] + f_ptr[5][idx] - f_ptr[6][idx] - f_ptr[7][idx] + f_ptr[8][idx]) * invRho_roi;
                    float uy_roi = (f_ptr[2][idx] - f_ptr[4][idx] + f_ptr[5][idx] + f_ptr[6][idx] - f_ptr[7][idx] - f_ptr[8][idx]) * invRho_roi;

                    // Run a second relaxation cycle to tighten boundary stability
                    float usqr_roi = 1.5f * (ux_roi * ux_roi + uy_roi * uy_roi);
                    for (int i = 0; i < 9; i++) {
                        float cu = 3.0f * (cxs[i] * ux_roi + cys[i] * uy_roi);
                        float feq_i = weights[i] * rho_roi * (1.0f + cu + 0.5f * cu * cu - usqr_roi);
                        // Blend current state with equilibrium again (Double relaxation)
                        f_ptr[i][idx] = f_ptr[i][idx] * 0.9f + feq_i * 0.1f;
                    }
                }
            }
        }

        // Finalize Time Step: Swap grid buffers
        for (int i = 0; i < 9; i++) f[i].swap(fNew[i]);
        return {(float)totalForceX, -(float)totalForceY};
    }
};

// --- JNI INTERFACE (Bridging C++ to Kotlin) ---

extern "C" JNIEXPORT jlong JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_initEngine(JNIEnv *env, jobject thiz, jint width, jint height) {
    return (jlong)(uintptr_t)new LBMEngine(width, height);
}
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_destroyEngine(JNIEnv *env, jobject thiz, jlong ptr) {
    delete reinterpret_cast<LBMEngine*>(ptr);
}

/**
 * Circle Drawing: Uses standard circle equation to mask the grid.
 * Also computes BFL 'q' values for smooth boundary reflection.
 */
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_addObstacleNative(JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint radius) {
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

/**
 * Box Drawing: Simplest obstacle type. Fixed q=0.5 (Staircase) for now.
 */
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_addBoxObstacleNative(JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint size) {
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

/**
 * NACA Airfoil Drawing: Implements the standard NACA 4-digit parametric chord equations.
 * Supports Angle of Attack (Degrees) and camber (m, p) / thickness (t).
 */
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_addNacaAirfoilNative(
        JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint chord, jfloat m, jfloat p, jfloat t, jfloat angleDegrees) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); if (chord <= 0) return;
    // In Android Y-down space, a positive rotation angleDegrees tilts the tail DOWN (Nose Up).
    // The sampling logic below uses [cos sin; -sin cos] which is a coordinate rotation of -angleRad.
    // This is equivalent to an object rotation of +angleRad.
    float angleRad = angleDegrees * (M_PI / 180.0f); float cosA = cosf(angleRad); float sinA = sinf(angleRad);
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

extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setDensityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat d) { reinterpret_cast<LBMEngine*>(ptr)->rhoAir = d; }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setViscosityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat v) { reinterpret_cast<LBMEngine*>(ptr)->nuAir = v; }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setDeltaTimeNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat dt) { reinterpret_cast<LBMEngine*>(ptr)->dt = dt; }

extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setLocalRefinementEnabledNative(JNIEnv *env, jobject thiz, jlong ptr, jboolean enabled) {
    reinterpret_cast<LBMEngine*>(ptr)->localRefinementEnabled = (bool)enabled;
}

/**
 * Headless Simulation Step: Used for high-speed background computations.
 */
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_stepNative(JNIEnv *env, jobject thiz, jlong ptr, jint steps) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr);
    if (!e) return;
    double dragAcc = 0.0; double liftAcc = 0.0;
    for (int i = 0; i < steps; i++) {
        ForceResult res = e->step();
        dragAcc += (double)res.drag;
        liftAcc += (double)res.lift;
    }
    if (steps > 0) {
        float avgD = (float)(dragAcc / steps); float avgL = (float)(liftAcc / steps);
        float conv = e->rhoAir * (e->dx * e->dx * e->dx) / (e->dt * e->dt);
        e->dragForceNewtons = fabsf(avgD) * conv; e->liftForceNewtons = avgL * conv;
        float uL = e->uInlet; float refL = fmaxf(e->frontalArea, e->horizontalSpan) / e->dx;
        float dynP = 0.5f * (uL * uL);
        if (uL > 0.001f && refL > 0.5f) {
            float instantCd = fabsf(avgD) / (dynP * refL);
            float instantCl = avgL / (dynP * refL);
            e->dragCoefficient = instantCd;
            e->liftCoefficient = instantCl;
            e->smoothedCd = e->smoothedCd * 0.98f + instantCd * 0.02f;
            e->smoothedCl = e->smoothedCl * 0.98f + instantCl * 0.02f;
        } else {
            e->dragCoefficient = 0; e->liftCoefficient = 0;
            e->smoothedCd = 0; e->smoothedCl = 0;
        }
    }
}

/**
 * FUSED PHYSICS & RENDER CALL.
 * Executes simulation and maps the result to the Android Bitmap in one JNI pass.
 */
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_stepAndRenderNative(JNIEnv *env, jobject thiz, jlong ptr, jobject bitmap, jint steps, jboolean drawBlack) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr); double dragAcc = 0.0; double liftAcc = 0.0;

    // 1. EXECUTE PHYSICS
    for (int i = 0; i < steps; i++) { ForceResult res = e->step(); dragAcc += (double)res.drag; liftAcc += (double)res.lift; }

    // 2. POST-PROCESS AERODYNAMIC COEFFICIENTS
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
            e->smoothedCd = e->smoothedCd * 0.98f + instantCd * 0.02f;
            e->smoothedCl = e->smoothedCl * 0.98f + instantCl * 0.02f;
        } else {
            e->dragCoefficient = 0; e->liftCoefficient = 0;
            e->smoothedCd = 0; e->smoothedCl = 0;
        }
    }

    // 3. RENDER DIRECTLY TO BITMAP PIXELS
    void* pixels; if (AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) return;
    auto* bmp = static_cast<uint32_t*>(pixels);
    int w = e->width; int h = e->height; float invMaxV = 255.0f / (e->uInlet * 1.8f);
    static const float invC[10] = { 0, 1.0f, 0.5f, 0.3333333f, 0.25f, 0.2f, 0.1666667f, 0.1428571f, 0.125f, 0.1111111f };
    const uint8_t* obs = e->obstacles.data(); const float* src = e->visualizationSource.data();
    float* sm = e->smoothedVelocityMag.data(); uint32_t* lut = e->colorLUT;

    // Determine color for obstacles to prevent blur-bleed in HD mode
    uint32_t obstacleColor = drawBlack ? 0xFF000000 : lut[0];

    // --- SPATIAL + TEMPORAL SMOOTHING PASS ---
    // Smooths out pixel aliasing and high-frequency numerical noise
#pragma omp parallel for default(none) shared(obs, src, sm, bmp, w, h, invMaxV, invC, lut, e, obstacleColor, drawBlack) schedule(static) \
    num_threads(e->numThreads)
    for (int y = 0; y < h; y++) {
        bool y_edge = (y == 0 || y == h - 1);
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;
            if (obs[idx]) {
                // HD MODE TRICK: To prevent "Blue Halo" interpolation artifacts,
                // we "dilate" the surrounding fluid colors into the obstacle area.
                if (!drawBlack) {
                    float neighborSum = 0;
                    int neighborCount = 0;
                    // Sample immediate neighbors to find local fluid color
                    if (y > 0 && !obs[idx-w]) { neighborSum += src[idx-w]; neighborCount++; }
                    if (y < h-1 && !obs[idx+w]) { neighborSum += src[idx+w]; neighborCount++; }
                    if (x > 0 && !obs[idx-1]) { neighborSum += src[idx-1]; neighborCount++; }
                    if (x < w-1 && !obs[idx+1]) { neighborSum += src[idx+1]; neighborCount++; }

                    // If we have fluid neighbors, use their average.
                    // If we are deep inside a large wing, use the inlet velocity as a neutral filler.
                    float dilatedV = (neighborCount > 0) ? (neighborSum / neighborCount) : e->uInlet;
                    sm[idx] = sm[idx] * 0.7f + dilatedV * 0.3f;
                    int ci = (int)(sm[idx] * ((e->vizMode == LBMEngine::VELOCITY) ? invMaxV : 255.0f));
                    bmp[idx] = lut[std::max(0, std::min(255, ci))];
                } else {
                    sm[idx] = 0;
                    bmp[idx] = 0xFF000000; // Standard Mode: Solid Black
                }
                continue;
            }

            float sum = 0;
            int count = 0;

            if (!y_edge && x > 0 && x < w - 1) {
                // High-performance interior pass (unrolled)
                int t = idx - w; int b = idx + w;
                sum = src[t-1]+src[t]+src[t+1]+src[idx-1]+src[idx]+src[idx+1]+src[b-1]+src[b]+src[b+1];
                count = (1-obs[t-1])+(1-obs[t])+(1-obs[t+1])+(1-obs[idx-1])+(1-obs[idx])+(1-obs[idx+1])+(1-obs[b-1])+(1-obs[b])+(1-obs[b+1]);
            } else {
                // Edge pass with bounds checking
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

            // Apply Temporal Damping (70% prev, 30% current) for smooth visuals
            float rv = sm[idx] * 0.7f + (sum * invC[count]) * 0.3f;
            sm[idx] = rv;
            int ci = (int)(rv * ((e->vizMode == LBMEngine::VELOCITY) ? invMaxV : 255.0f));
            bmp[idx] = lut[std::max(0, std::min(255, ci))];
        }
    }
    AndroidBitmap_unlockPixels(env, bitmap);
}

extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_resetSimulationNative(JNIEnv *env, jobject thiz, jlong ptr) { reinterpret_cast<LBMEngine*>(ptr)->reset(); }

extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setInletVelocityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat v) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr);
    float uL = v * e->dt / e->dx;
    e->uInletTarget = fmaxf(0.0f, fminf(0.45f, uL));
}

extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getInletVelocityNative(JNIEnv *env, jobject thiz, jlong ptr) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr);
    return e->uInlet * e->dx / e->dt;
}

extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getDragForceNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->dragForceNewtons; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getDragCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->smoothedCd;
}
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getInstantDragCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->dragCoefficient;
}
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getLiftForceNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->liftForceNewtons; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getLiftCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->smoothedCl;
}
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getInstantLiftCoefficientNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->liftCoefficient;
}
extern "C" JNIEXPORT jlong JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getTotalStepsNative(JNIEnv *env, jobject thiz, jlong ptr) { return (jlong)reinterpret_cast<LBMEngine*>(ptr)->totalSteps; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getDensityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->rhoAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getViscosityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->nuAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getDXNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->dx; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getHorizontalSpanNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->horizontalSpan; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getFrontalAreaNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->frontalArea; }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setVisualizationModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint m) { reinterpret_cast<LBMEngine*>(ptr)->vizMode = static_cast<LBMEngine::VisualizationMode>(m); }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setBoundaryModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint mode) {
    reinterpret_cast<LBMEngine*>(ptr)->boundaryMode = static_cast<LBMEngine::BoundaryMode>(mode);
}

extern "C" JNIEXPORT jint JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getMaxCoresNative(JNIEnv *env, jobject thiz) {
    return omp_get_num_procs();
}

extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setNumThreadsNative(JNIEnv *env, jobject thiz, jlong ptr, jint n) {
    if (n > 0) reinterpret_cast<LBMEngine*>(ptr)->numThreads = n;
}

extern "C" JNIEXPORT jint JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getActiveCoresNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->activeCores;
}
