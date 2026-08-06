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
 * Color Scheme Mapper.
 * Maps normalized fluid property (0.0 to 1.0) to various high-contrast scales.
 */
inline uint32_t mapColor(float value, int scheme) {
    value = fmaxf(0.0f, fminf(1.0f, value));
    float r = 0.0f, g = 0.0f, b = 0.0f;

    switch (scheme) {
        case 0: // Standard (Jet)
            if (value < 0.25f) { r = 0.0f; g = 4.0f * value; b = 1.0f; }
            else if (value < 0.5f) { r = 0.0f; g = 1.0f; b = 1.0f - 4.0f * (value - 0.25f); }
            else if (value < 0.75f) { r = 4.0f * (value - 0.5f); g = 1.0f; b = 0.0f; }
            else { r = 1.0f; g = 1.0f - 4.0f * (value - 0.75f); b = 0.0f; }
            break;
        case 1: // Ironbow (Heat)
            r = fminf(1.0f, value * 3.0f);
            g = fmaxf(0.0f, fminf(1.0f, (value - 0.3f) * 3.0f));
            b = fmaxf(0.0f, fminf(1.0f, (value - 0.7f) * 3.0f));
            break;
        case 2: // Grayscale
            r = g = b = value;
            break;
        case 3: // Biolume (Ocean)
            r = fmaxf(0.0f, (value - 0.7f) * 3.3f);
            g = fmaxf(0.0f, fminf(1.0f, value * 1.5f));
            b = fminf(1.0f, value * 5.0f);
            break;
        default:
            r = g = b = value;
    }
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
    enum BoundaryMode { PERIODIC = 0, OPEN = 1 };

    int width, height;
    float uInlet = 0.06f;       // Current inlet velocity in lattice units
    float uInletTarget = 0.06f; // Target velocity for smoothing
    const float cs2 = 1.0f / 3.0f; // Speed of sound squared
    const float SmagorinskyConstant = 0.16f; // Subgrid turbulence parameter

    // Physical calibration constants
    float dx = 0.0025f;     // Grid spacing (meters)
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
    int numThreads = 4;
    int activeCores = 0;
    float frontalArea = 0.0f;
    float horizontalSpan = 0.0f;
    unsigned long long totalSteps = 0;
    bool aeroValid = true;

    VisualizationMode vizMode = VELOCITY;
    BoundaryMode boundaryMode = PERIODIC;
    int colorScheme = 0;

    // Distribution functions (Stored as Structure-of-Arrays for SIMD optimization)
    std::vector<float> f[9];
    std::vector<float> fNew[9];
    std::vector<uint8_t> obstacles;
    std::vector<float> linkQ[9]; // BFL: fractional distance to actual curved wall

    std::vector<float> ux_flow, uy_flow;
    std::vector<float> velocityMag;
    std::vector<float> visualizationSource;
    std::vector<float> smoothedVelocityMag;

    uint32_t colorLUT[256];

    // D2Q9 Lattice Vectors
    const float cxs[9] = {0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 1.0f, -1.0f, -1.0f, 1.0f};
    const float cys[9] = {0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f};
    const float weights[9] = {4.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/9, 1.f/36, 1.f/36, 1.f/36, 1.f/36};
    const int opposite[9] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

    LBMEngine(int w, int h) : width(w), height(h) {
        updateColorLUT(0);
        int numProcs = omp_get_num_procs();
        numThreads = (numProcs >= 8 ? 4 : numProcs);
        int size = width * height;
        for (int i = 0; i < 9; i++) {
            f[i].resize(size, 0.0f);
            fNew[i].resize(size, 0.0f);
            linkQ[i].resize(size, 0.5f);
        }
        obstacles.resize(size, 0);
        ux_flow.resize(size, 0.0f);
        uy_flow.resize(size, 0.0f);
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
            ux_flow[idx] = velocity;
            uy_flow[idx] = 0;
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
        std::fill(ux_flow.begin(), ux_flow.end(), 0.0f);
        std::fill(uy_flow.begin(), uy_flow.end(), 0.0f);
        std::fill(velocityMag.begin(), velocityMag.end(), 0.0f);
        std::fill(visualizationSource.begin(), visualizationSource.end(), 0.0f);
        std::fill(smoothedVelocityMag.begin(), smoothedVelocityMag.end(), 0.0f);
        totalSteps = 0;
        smoothedCd = 0.0f;
        smoothedCl = 0.0f;
        frontalArea = 0.0f;
        horizontalSpan = 0.0f;
        aeroValid = true;
        initFluid(uInletTarget);
    }

    /**
     * CORE SIMULATION KERNEL.
     * Executes a single Time Step. Fused logic significantly improves cache performance.
     */
    ForceResult step(bool updateViz) {
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

        float* __restrict ux_f = ux_flow.data();
        float* __restrict uy_f = uy_flow.data();

#pragma omp parallel for schedule(static) default(none) \
    shared(f_ptr, fn_ptr, obs_ptr, lq_ptr, ux_f, uy_f, velocityMag, visualizationSource, width, height, cs2, smagCoeff, inv2cs2, tau_0, regFactor, weights, CX, CY, opposite, vizMode, uInlet, numThreads, boundaryMode, W0, W1, W2, updateViz) \
    reduction(+:totalForceX, totalForceY) reduction(min:stepMinY, stepMinX) reduction(max:stepMaxY, stepMaxX) \
    num_threads(numThreads)
        for (int y = 0; y < height; y++) {
            if (omp_get_thread_num() == 0) activeCores = omp_get_num_threads();
            for (int x = 0; x < width; x++) {
                const int idx = y * width + x;
                if (obs_ptr[idx]) {
                    if (y < stepMinY) stepMinY = y; if (y > stepMaxY) stepMaxY = y;
                    if (x < stepMinX) stepMinX = x; if (x > stepMaxX) stepMaxX = x;
                    continue;
                }

                float local_f[9];
                bool touchesWall = false;

                if (x == 0) {
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
                } else if (x == width - 1) {
                    // TRULY OPEN Outflow: Equilibrium-Preserving Zero-Gradient
                    // We copy the non-equilibrium part of the populations from the neighbor.
                    // This allows waves and vortices to "stream" out of the tunnel at the local velocity
                    // without any artificial pressure boundary or energy absorption.
                    const int prev = idx - 1;
                    float rho_p = f_ptr[0][prev] + f_ptr[1][prev] + f_ptr[2][prev] + f_ptr[3][prev] + f_ptr[4][prev] + f_ptr[5][prev] + f_ptr[6][prev] + f_ptr[7][prev] + f_ptr[8][prev];
                    float invRhoP = 1.0f / fmaxf(0.1f, rho_p);
                    float ux_p = (f_ptr[1][prev] - f_ptr[3][prev] + f_ptr[5][prev] - f_ptr[6][prev] - f_ptr[7][prev] + f_ptr[8][prev]) * invRhoP;
                    float uy_p = (f_ptr[2][prev] - f_ptr[4][prev] + f_ptr[5][prev] + f_ptr[6][prev] - f_ptr[7][prev] - f_ptr[8][prev]) * invRhoP;
                    float u2sq_p = 1.5f * (ux_p * ux_p + uy_p * uy_p);

                    for (int i = 0; i < 9; i++) {
                        float cu_p = 3.0f * (cxs[i] * ux_p + cys[i] * uy_p);
                        float feq_p = weights[i] * rho_p * (1.0f + cu_p + 0.5f * cu_p * cu_p - u2sq_p);
                        float fneq_p = f_ptr[i][prev] - feq_p;
                        // Use the exact density and velocity of the neighbor to eliminate the impedance mismatch
                        float feq_out = weights[i] * rho_p * (1.0f + cu_p + 0.5f * cu_p * cu_p - u2sq_p);
                        local_f[i] = feq_out + fneq_p;
                    }
                } else {
                    for (int i = 0; i < 9; i++) {
                        int nx = x - CX[i]; int ny = y - CY[i];
                        if (nx < 0) nx = width - 1; else if (nx >= width) nx = 0;
                        if (ny < 0 || ny >= height) {
                            if (boundaryMode == PERIODIC) { if (ny < 0) ny = height - 1; else ny = 0; }
                            else { // OPEN (Far-Field)
                                // Far-Field Boundary (Infinity): Pin to ambient state (rho=1.0, u=uInlet)
                                // but keep the non-equilibrium part to allow pressure waves to pass through.
                                int ny_n = (ny < 0) ? 0 : height - 1;
                                int idx_n = ny_n * width + x;

                                float rho_n = f_ptr[0][idx_n] + f_ptr[1][idx_n] + f_ptr[2][idx_n] + f_ptr[3][idx_n] + f_ptr[4][idx_n] + f_ptr[5][idx_n] + f_ptr[6][idx_n] + f_ptr[7][idx_n] + f_ptr[8][idx_n];
                                float invRhoN = 1.0f / fmaxf(0.1f, rho_n);
                                float ux_n = (f_ptr[1][idx_n] - f_ptr[3][idx_n] + f_ptr[5][idx_n] - f_ptr[6][idx_n] - f_ptr[7][idx_n] + f_ptr[8][idx_n]) * invRhoN;
                                float uy_n = (f_ptr[2][idx_n] - f_ptr[4][idx_n] + f_ptr[5][idx_n] + f_ptr[6][idx_n] - f_ptr[7][idx_n] - f_ptr[8][idx_n]) * invRhoN;

                                float u2sq_n = 1.5f * (ux_n * ux_n + uy_n * uy_n);
                                float cu_n = 3.0f * (cxs[i] * ux_n + cys[i] * uy_n);
                                float feq_n = weights[i] * rho_n * (1.0f + cu_n + 0.5f * cu_n * cu_n - u2sq_n);

                                // Target Far-Field Equilibrium (ambient state)
                                float u2sq_inf = 1.5f * (uInlet * uInlet);
                                float cu_inf = 3.0f * (cxs[i] * uInlet);
                                float feq_inf = weights[i] * 1.0f * (1.0f + cu_inf + 0.5f * cu_inf * cu_inf - u2sq_inf);

                                local_f[i] = feq_inf + (f_ptr[i][idx_n] - feq_n);
                                continue;
                            }
                        }
                        const int srcIdx = ny * width + nx;
                        if (obs_ptr[srcIdx]) {
                            touchesWall = true;
                            const int dir_to_wall = opposite[i];
                            const float q = lq_ptr[dir_to_wall][idx];
                            const float fi_out = f_ptr[dir_to_wall][idx];
                            float fi_in;
                            if (q < 0.5f) {
                                int nx2 = x + CX[i]; int ny2 = y + CY[i];
                                if (nx2 < 0) nx2 = width - 1; else if (nx2 >= width) nx2 = 0;
                                if (ny2 < 0) ny2 = height - 1; else if (ny2 >= height) ny2 = 0;
                                const int idx2 = ny2 * width + nx2;
                                if (obs_ptr[idx2]) fi_in = q * 2.0f * fi_out + (1.0f - 2.0f * q) * f_ptr[i][idx];
                                else fi_in = 2.0f * q * fi_out + (1.0f - 2.0f * q) * f_ptr[dir_to_wall][idx2];
                            } else {
                                const float inv2q = 1.0f / (2.0f * q);
                                fi_in = inv2q * fi_out + (1.0f - inv2q) * f_ptr[i][idx];
                            }
                            local_f[i] = fi_in;
                            totalForceX += (double)(fi_out + fi_in) * (double)CX[dir_to_wall];
                            totalForceY += (double)(fi_out + fi_in) * (double)CY[dir_to_wall];
                        } else local_f[i] = f_ptr[i][srcIdx];
                    }
                }

                const float rho = local_f[0] + local_f[1] + local_f[2] + local_f[3] + local_f[4] + local_f[5] + local_f[6] + local_f[7] + local_f[8];
                const float invRho = 1.0f / fmaxf(0.1f, rho);
                float ux = (local_f[1] - local_f[3] + local_f[5] - local_f[6] - local_f[7] + local_f[8]) * invRho;
                float uy = (local_f[2] - local_f[4] + local_f[5] + local_f[6] - local_f[7] - local_f[8]) * invRho;
                float vMag2 = ux * ux + uy * uy;
                if (vMag2 > 0.2025f) { float s = 0.45f / sqrtf(vMag2); ux *= s; uy *= s; vMag2 = 0.2025f; }

                const float row_w_9 = rho * W1; const float row_w_36 = rho * W2;
                const float one_minus_15u2 = 1.0f - 1.5f * vMag2;
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

                float nu_t = 0;
                if (x > 0 && x < width - 1 && y > 0 && y < height - 1) {
                    float g11 = (ux_f[idx + 1] - ux_f[idx - 1]) * 0.5f;
                    float g12 = (ux_f[idx + width] - ux_f[idx - width]) * 0.5f;
                    float g21 = (uy_f[idx + 1] - uy_f[idx - 1]) * 0.5f;
                    float g22 = (uy_f[idx + width] - uy_f[idx - width]) * 0.5f;
                    float sd11 = g11 * g11 + g12 * g21; float sd22 = g21 * g12 + g22 * g22;
                    float sd12 = 0.5f * (g11 * g12 + g12 * g22 + g21 * g11 + g22 * g21);
                    float trSd = sd11 + sd22; float Sd11 = sd11 - 0.5f * trSd; float Sd22 = sd22 - 0.5f * trSd;
                    float Sd2 = Sd11 * Sd11 + Sd22 * Sd22 + 2.0f * sd12 * sd12;
                    float S2 = g11 * g11 + g22 * g22 + 0.5f * (g12 + g21) * (g12 + g21);
                    if (S2 > 1e-10f) {
                        const float cw = 0.325f;
                        float sd2_15 = Sd2 * sqrtf(Sd2);
                        float s2_25 = S2 * S2 * sqrtf(S2);
                        float sd2_125 = Sd2 * sqrtf(sqrtf(Sd2)); // Correct x^1.25 scaling
                        nu_t = (cw * cw) * sd2_15 / (s2_25 + sd2_125);
                    }
                }

                const float tau_eff = tau_0 + 3.0f * nu_t;
                const float one_minus_invTau = 1.0f - (1.0f / fmaxf(tau_eff, 0.505f));
                const float common_reg = one_minus_invTau * regFactor;
                const float reg_pixx = common_reg * (pixx * (1.0f - cs2) - piyy * cs2);
                const float reg_piyy = common_reg * (-pixx * cs2 + piyy * (1.0f - cs2));
                const float reg_pixy = common_reg * 2.0f * pixy;
                const float reg_diag = 2.0f * (reg_pixx + reg_piyy);

                fn_ptr[0][idx] = feq[0] + W0 * common_reg * (-cs2 * (pixx + piyy));
                fn_ptr[1][idx] = feq[1] + W1 * reg_pixx; fn_ptr[2][idx] = feq[2] + W1 * reg_piyy;
                fn_ptr[3][idx] = feq[3] + W1 * reg_pixx; fn_ptr[4][idx] = feq[4] + W1 * reg_piyy;
                fn_ptr[5][idx] = feq[5] + W2 * (reg_diag + reg_pixy); fn_ptr[6][idx] = feq[6] + W2 * (reg_diag - reg_pixy);
                fn_ptr[7][idx] = feq[7] + W2 * (reg_diag + reg_pixy); fn_ptr[8][idx] = feq[8] + W2 * (reg_diag - reg_pixy);

                ux_f[idx] = ux; uy_f[idx] = uy;

                if (updateViz) {
                    float vMag = sqrtf(vMag2); velocityMag[idx] = vMag;
                    const float ps = (rho - 1.0f) * cs2;
                    const float pressScale = (uInlet > 0.001f) ? (1.0f / (uInlet * uInlet)) : 100.0f;
                    if (vizMode == VELOCITY) visualizationSource[idx] = vMag;
                    else if (vizMode == PRESSURE) visualizationSource[idx] = 0.5f + ps * pressScale;
                    else {
                        // Total Pressure: Static + Dynamic Pressure.
                        // We subtract the ambient total pressure (0.5 * uInlet^2) to center the map on 0.5.
                        float p_total = ps + 0.5f * rho * vMag2;
                        float p_total_ambient = 0.5f * uInlet * uInlet;
                        visualizationSource[idx] = 0.5f + (p_total - p_total_ambient) * pressScale;
                    }
                }
            }
        }

        // Finalize aerodynamic geometric stats
        for (int i = 0; i < 9; i++) f[i].swap(fNew[i]);

        // Automatic Area Calculation: Update reference dimensions from obstacle bounding box
        // if they haven't been precisely set (e.g. by NACA call).
        if (stepMaxY >= stepMinY) {
            float boxArea = static_cast<float>(stepMaxY - stepMinY + 1) * dx;
            float boxSpan = static_cast<float>(stepMaxX - stepMinX + 1) * dx;
            // Use fmax to handle multiple obstacles added sequentially
            frontalArea = fmaxf(frontalArea, boxArea);
            horizontalSpan = fmaxf(horizontalSpan, boxSpan);
        }

        // Lift/Drag Not Applicable if obstacle touches the top/bottom boundary
        // especially in Infinity mode where momentum exchange is truncated.
        aeroValid = (stepMinY > 0 && stepMaxY < height - 1);

        return {(float)totalForceX, -(float)totalForceY};
    }

    /**
     * Rescales the entire fluid grid to maintain physical consistency when
     * the simulation scale (dx) changes.
     */
    void rescaleVelocity(float factor) {
        if (factor == 1.0f || factor <= 0.0f) return;
        #pragma omp parallel for schedule(static) default(none) shared(width, height, factor, obstacles, f, fNew, ux_flow, uy_flow, weights, cxs, cys, numThreads)
        for (int idx = 0; idx < width * height; idx++) {
            if (obstacles[idx]) continue;
            float rho = 0;
            for (int i = 0; i < 9; i++) rho += f[i][idx];
            float invRho = 1.0f / fmaxf(0.1f, rho);
            float ux = (f[1][idx] - f[3][idx] + f[5][idx] - f[6][idx] - f[7][idx] + f[8][idx]) * invRho;
            float uy = (f[2][idx] - f[4][idx] + f[5][idx] + f[6][idx] - f[7][idx] - f[8][idx]) * invRho;

            float ux_n = ux * factor;
            float uy_n = uy * factor;
            ux_flow[idx] = ux_n; uy_flow[idx] = uy_n;

            float u2sq_o = 1.5f * (ux * ux + uy * uy);
            float u2sq_n = 1.5f * (ux_n * ux_n + uy_n * uy_n);

            for (int i = 0; i < 9; i++) {
                float cu_o = 3.0f * (cxs[i] * ux + cys[i] * uy);
                float cu_n = 3.0f * (cxs[i] * ux_n + cys[i] * uy_n);
                float feq_o = weights[i] * rho * (1.0f + cu_o + 0.5f * cu_o * cu_o - u2sq_o);
                float feq_n = weights[i] * rho * (1.0f + cu_n + 0.5f * cu_n * cu_n - u2sq_n);
                f[i][idx] = feq_n + (f[i][idx] - feq_o);
                fNew[i][idx] = f[i][idx];
            }
        }
    }

    void updateColorLUT(int scheme) {
        colorScheme = scheme;
        for (int i = 0; i < 256; i++) {
            colorLUT[i] = mapColor(static_cast<float>(i) / 255.0f, scheme);
        }
    }
};

/**
 * Helper to update aerodynamic coefficients in the engine state.
 */
void updateAerodynamics(LBMEngine* e, double dragAcc, double liftAcc, int steps) {
    if (steps <= 0) return;
    float avgD = static_cast<float>(dragAcc / steps);
    float avgL = static_cast<float>(liftAcc / steps);
    float conv = e->rhoAir * (e->dx * e->dx * e->dx) / (e->dt * e->dt);
    e->dragForceNewtons = fabsf(avgD) * conv;
    e->liftForceNewtons = avgL * conv;
    float uL = e->uInlet;
    // Aerodynamic Convention:
    // For wings/streamlined bodies, use Chord (horizontal span).
    // For blunt bodies (cylinders/boxes), use Frontal Area (height).
    // We take the maximum of the two to automatically select the most appropriate reference.
    float refL = fmaxf(e->frontalArea, e->horizontalSpan) / e->dx;
    float dynP = 0.5f * (uL * uL);
    if (e->aeroValid && uL > 0.001f && refL > 0.5f) {
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

// --- JNI INTERFACE (Bridging C++ to Kotlin) ---

extern "C" JNIEXPORT jlong JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_initEngine(JNIEnv *env, jobject thiz, jint width, jint height) {
    return static_cast<jlong>(reinterpret_cast<uintptr_t>(new LBMEngine(width, height)));
}
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_destroyEngine(JNIEnv *env, jobject thiz, jlong ptr) {
    delete reinterpret_cast<LBMEngine*>(ptr);
}

/**
 * Circle Drawing: Uses standard circle equation to mask the grid.
 * Also computes BFL 'q' values for smooth boundary reflection.
 */
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_addObstacleNative(JNIEnv *env, jobject thiz, jlong ptr, jint cx, jint cy, jint radius) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr);
    float R = static_cast<float>(radius);
    float r2_threshold = R * R - 0.1f; // Slightly tighter bound to avoid single cardinal pixels

    for (int y = std::max(0, cy - radius); y <= std::min(e->height - 1, cy + radius); y++) {
        for (int x = std::max(0, cx - radius); x <= std::min(e->width - 1, cx + radius); x++) {
            float dx = static_cast<float>(x - cx);
            float dy = static_cast<float>(y - cy);
            if (dx * dx + dy * dy <= r2_threshold) e->obstacles[y * e->width + x] = 1;
        }
    }
    for (int y = std::max(0, cy - radius - 1); y <= std::min(e->height - 1, cy + radius + 1); y++) {
        for (int x = std::max(0, cx - radius - 1); x <= std::min(e->width - 1, cx + radius + 1); x++) {
            int idx = y * e->width + x; if (e->obstacles[idx]) continue;
            float dxf = (float)(x - cx); float dyf = (float)(y - cy);
            for (int i = 1; i < 9; i++) {
                int nx = x + (int)e->cxs[i]; int ny = y + (int)e->cys[i];
                if (nx < 0 || nx >= e->width || ny < 0 || ny >= e->height) continue;
                if (e->obstacles[ny * e->width + nx]) {
                    float vx = e->cxs[i]; float vy = e->cys[i];
                    float A = vx*vx + vy*vy; float B = 2.0f * (dxf*vx + dyf*vy); float C = dxf*dxf + dyf*dyf - (R*R - 0.1f);
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
    float angleRad = angleDegrees * static_cast<float>(M_PI / 180.0);
    float cosA = cosf(angleRad); float sinA = sinf(angleRad);
    int pad = chord / 2; int x_min = std::max(0, cx - pad); int x_max = std::min(e->width - 1, cx + chord + pad);
    int y_min = std::max(0, cy - chord); int y_max = std::min(e->height - 1, cy + chord);
    // 1. Helper to check if a global coordinate is inside the NACA profile
    // Uses the high-fidelity perpendicular thickness definition
    auto isInside = [&](float px, float py) -> bool {
        float fChord = static_cast<float>(chord);
        float dx_rel = px - (static_cast<float>(cx) + 0.5f);
        float dy_rel = py - (static_cast<float>(cy) + 0.5f);

        // Coordinate rotation to local wing space
        float x_l = dx_rel * cosA + dy_rel * sinA;
        float y_l = -dx_rel * sinA + dy_rel * cosA;

        float x_f = x_l / fChord;
        if (x_f < 0.0f || x_f > 1.0f) return false;

        // Camber and Slope
        float yc = 0.0f;
        float dyc_dx = 0.0f;
        if (p > 0.001f) {
            if (x_f <= p) {
                yc = (m / (p * p)) * (2.0f * p * x_f - x_f * x_f);
                dyc_dx = (2.0f * m / (p * p)) * (p - x_f);
            } else {
                float one_m_p_sq = (1.0f - p) * (1.0f - p);
                yc = (m / one_m_p_sq) * ((1.0f - 2.0f * p) + 2.0f * p * x_f - x_f * x_f);
                dyc_dx = (2.0f * m / one_m_p_sq) * (p - x_f);
            }
        }

        // Thickness (using -0.1036 coefficient for exact closure at the trailing edge)
        float yt = 5.0f * t * (0.2969f * sqrtf(x_f) - 0.1260f * x_f - 0.3516f * x_f * x_f + 0.2843f * powf(x_f, 3) - 0.1036f * powf(x_f, 4));

        // Exact Perpendicular Distance Check
        // theta is the angle of the camber line
        float theta = atanf(dyc_dx);
        // The distance from the point to the camber line, projected onto the normal
        // Note: y_l is positive down in local space, yc is positive up in NACA definition
        float dist_to_camber = (y_l + yc * fChord) * cosf(theta);

        return fabsf(dist_to_camber) <= yt * fChord;
    };

    // 2. Render the visual mask with 8x Super-Sampling to eliminate single-pixel spikes
    for (int py = y_min; py <= y_max; py++) {
        for (int px = x_min; px <= x_max; px++) {
            int insideCount = 0;
            for (int sy = 0; sy < 8; sy++) {
                for (int sx = 0; sx < 8; sx++) {
                    if (isInside(static_cast<float>(px) + (sx + 0.5f) / 8.0f, static_cast<float>(py) + (sy + 0.5f) / 8.0f))
                        insideCount++;
                }
            }
            if (insideCount >= 32) e->obstacles[py * e->width + px] = 1;
        }
    }

    // 3. Compute accurate sub-grid distances (q) for BFL boundary scheme
    // We calculate q relative to the TRUE mathematical boundary, ignoring the pixel mask
    for (int py = std::max(0, y_min - 1); py <= std::min(e->height - 1, y_max + 1); py++) {
        for (int px = std::max(0, x_min - 1); px <= std::min(e->width - 1, x_max + 1); px++) {
            int idx = py * e->width + px;
            if (e->obstacles[idx]) continue;

            for (int i = 1; i < 9; i++) {
                float nx = static_cast<float>(px) + 0.5f + e->cxs[i];
                float ny = static_cast<float>(py) + 0.5f + e->cys[i];

                if (isInside(nx, ny)) {
                    // Intersection search: Solve for q relative to the true smooth curve
                    float q_low = 0.0f, q_high = 1.0f;
                    for (int iter = 0; iter < 12; iter++) {
                        float q_mid = (q_low + q_high) * 0.5f;
                        if (isInside(static_cast<float>(px) + 0.5f + q_mid * e->cxs[i],
                                     static_cast<float>(py) + 0.5f + q_mid * e->cys[i])) q_high = q_mid;
                        else q_low = q_mid;
                    }
                    e->linkQ[i][idx] = fmaxf(0.01f, q_high);
                }
            }
        }
    }

    // 4. Set Mathematical Reference Area for stable Cd/Cl readings
    float t_max = t * static_cast<float>(chord);
    // Approximate projected frontal area: thickness * cos(AoA) + chord * sin(AoA)
    e->frontalArea = (t_max * fabsf(cosf(angleRad)) + static_cast<float>(chord) * fabsf(sinf(angleRad))) * e->dx;
    e->horizontalSpan = static_cast<float>(chord) * fabsf(cosf(angleRad)) * e->dx;
}

extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setDensityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat d) { reinterpret_cast<LBMEngine*>(ptr)->rhoAir = d; }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setViscosityNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat v) { reinterpret_cast<LBMEngine*>(ptr)->nuAir = v; }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setDeltaTimeNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat dt) { reinterpret_cast<LBMEngine*>(ptr)->dt = dt; }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setDXNative(JNIEnv *env, jobject thiz, jlong ptr, jfloat dx) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr);
    float old_dx = e->dx;
    e->dx = dx;
    // Rescale the current fluid velocity in the grid to maintain physical consistency
    // New_u_lattice = Old_u_lattice * (Old_dx / New_dx)
    if (old_dx > 0 && dx > 0) {
        float factor = old_dx / dx;
        e->rescaleVelocity(factor);
        e->uInlet = e->uInlet * factor; // Instantly jump inlet state to avoid normalization lag
    }
}



/**
 * Headless Simulation Step: Used for high-speed background computations.
 */
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_stepNative(JNIEnv *env, jobject thiz, jlong ptr, jint steps) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr);
    if (!e) return;
    double dragAcc = 0.0; double liftAcc = 0.0;
    for (int i = 0; i < steps; i++) {
        ForceResult res = e->step(false); // Headless never needs viz preparation
        dragAcc += static_cast<double>(res.drag);
        liftAcc += static_cast<double>(res.lift);
    }
    updateAerodynamics(e, dragAcc, liftAcc, steps);
}

/**
 * FUSED PHYSICS & RENDER CALL.
 * Executes simulation and maps the result to the Android Bitmap in one JNI pass.
 */
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_stepAndRenderNative(JNIEnv *env, jobject thiz, jlong ptr, jobject bitmap, jint steps, jboolean drawBlack) {
    auto* e = reinterpret_cast<LBMEngine*>(ptr);
    double dragAcc = 0.0; double liftAcc = 0.0;

    // 1. EXECUTE PHYSICS
    for (int i = 0; i < steps; i++) {
        // Only prepare visualization data on the very last step to save CPU cycles
        ForceResult res = e->step(i == steps - 1);
        dragAcc += static_cast<double>(res.drag);
        liftAcc += static_cast<double>(res.lift);
    }

    // 2. POST-PROCESS AERODYNAMIC COEFFICIENTS
    updateAerodynamics(e, dragAcc, liftAcc, steps);

    // 3. RENDER DIRECTLY TO BITMAP PIXELS
    void* pixels; if (AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) return;
    auto* bmp = static_cast<uint32_t*>(pixels);
    int w = e->width; int h = e->height;

    // Normalization: Use the Target Velocity as the fixed reference for the color scale.
    // This ensures that colors remain stable when dx or speed change.
    float invMaxV = 255.0f / fmaxf(0.001f, e->uInletTarget * 1.8f);

    static const float invC[10] = { 0, 1.0f, 0.5f, 0.3333333f, 0.25f, 0.2f, 0.1666667f, 0.1428571f, 0.125f, 0.1111111f };
    const uint8_t* obs = e->obstacles.data(); const float* src = e->visualizationSource.data();
    float* sm = e->smoothedVelocityMag.data(); uint32_t* lut = e->colorLUT;

    // --- SPATIAL + TEMPORAL SMOOTHING PASS ---
    // Smooths out pixel aliasing and high-frequency numerical noise
#pragma omp parallel for default(none) shared(obs, src, sm, bmp, w, h, invMaxV, invC, lut, e, drawBlack) schedule(static) \
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
                    // Interior nodes use 0.0f (stationary) to ensure the boundary layer
                    // correctly interpolates toward a no-slip condition.
                    float dilatedV = (neighborCount > 0) ? (neighborSum / neighborCount) : 0.0f;
                    sm[idx] = sm[idx] * 0.7f + dilatedV * 0.3f;

                    if (e->colorScheme != 0) {
                        bmp[idx] = 0xFFFFFFFF; // Non-standard schemes: solid white obstacles
                    } else {
                        // Standard scheme: Use black obstacles to provide high contrast.
                        // This prevents obstacles from appearing Blue (lut[0]) in HD mode.
                        bmp[idx] = 0xFF000000;
                    }
                } else {
                    sm[idx] = 0;
                    bmp[idx] = (e->colorScheme == 0) ? 0xFF000000 : 0xFFFFFFFF;
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
extern "C" JNIEXPORT jboolean JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_isAerodynamicsValidNative(JNIEnv *env, jobject thiz, jlong ptr) {
    return reinterpret_cast<LBMEngine*>(ptr)->aeroValid;
}
extern "C" JNIEXPORT jlong JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getTotalStepsNative(JNIEnv *env, jobject thiz, jlong ptr) { return (jlong)reinterpret_cast<LBMEngine*>(ptr)->totalSteps; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getDensityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->rhoAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getViscosityNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->nuAir; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getDXNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->dx; }
extern "C" JNIEXPORT jfloat JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_getHorizontalSpanNative(JNIEnv *env, jobject thiz, jlong ptr) { return reinterpret_cast<LBMEngine*>(ptr)->horizontalSpan; }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setVisualizationModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint m) { reinterpret_cast<LBMEngine*>(ptr)->vizMode = static_cast<LBMEngine::VisualizationMode>(m); }
extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setBoundaryModeNative(JNIEnv *env, jobject thiz, jlong ptr, jint mode) {
    reinterpret_cast<LBMEngine*>(ptr)->boundaryMode = static_cast<LBMEngine::BoundaryMode>(mode);
}

extern "C" JNIEXPORT void JNICALL Java_com_bc_fluidsandbox_NativeLBMEngine_setColorSchemeNative(JNIEnv *env, jobject thiz, jlong ptr, jint scheme) {
    reinterpret_cast<LBMEngine*>(ptr)->updateColorLUT(scheme);
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
