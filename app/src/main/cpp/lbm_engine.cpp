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
    float omega = 1.0f / 0.53f;
    const float cs2 = 1.0f / 3.0f;
    const float SmagorinskyConstant = 0.12f;

    std::vector<float> f;
    std::vector<float> fNew;
    std::vector<bool> obstacles;
    std::vector<float> velocityMag;

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

        initFluid(uInlet);
    }

    void initFluid(float velocity) {
        float usqr = 1.5f * (velocity * velocity);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int idx = (y * width + x) * 9;
                for (int i = 0; i < 9; i++) {
                    float cu = 3.0f * (cxs[i] * velocity);
                    f[idx + i] = weights[i] * (1.0f + cu + 0.5f * cu * cu - usqr);
                }
            }
        }
        fNew = f;
    }

    void reset() {
        std::fill(obstacles.begin(), obstacles.end(), false);
        std::fill(velocityMag.begin(), velocityMag.end(), 0.0f);
        initFluid(uInlet);
    }

    void step() {
        // 0. Smooth inlet velocity to prevent numerical shocks
        uInlet = uInlet * 0.99f + uInletTarget * 0.01f;

        // 1. Streaming (Pull Method for Cache Efficiency)
#pragma omp parallel for schedule(static)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                for (int i = 0; i < 9; i++) {
                    int nx = x - cxs[i];
                    int ny = y - cys[i];
                    if (nx < 0) nx = width - 1; else if (nx >= width) nx = 0;
                    if (ny < 0) ny = height - 1; else if (ny >= height) ny = 0;

                    fNew[(y * width + x) * 9 + i] = f[(ny * width + nx) * 9 + i];
                }
            }
        }

        // 2. Collision with Regularization & Smagorinsky
#pragma omp parallel for schedule(static)
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
                        ux += fi * cxs[i];
                        uy += fi * cys[i];
                    }

                    // Numerical Safeguards: Clamp density
                    rho = std::max(0.1f, rho);
                    ux /= rho; uy /= rho;

                    if (x == 0) { ux = uInlet; uy = 0.0f; rho = 1.0f; }

                    // Numerical Safeguards: Clamp velocity magnitude
                    float vMag = std::sqrt(ux * ux + uy * uy);
                    if (vMag > 0.25f) {
                        float scale = 0.25f / vMag;
                        ux *= scale; uy *= scale;
                        vMag = 0.25f;
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
                        float fi_neq_reg = (weights[i] / (2.0f * cs2 * cs2)) * (Qixx * pixx + 2.0f * Qixy * pixy + Qiyy * piyy);

                        f[fBase + i] = fi_eq + (1.0f - omega_eff) * fi_neq_reg;
                    }

                    // 3. Sponge Layer Damping (Right Outlet)
                    if (x > width * 0.85) {
                        float spongeStart = width * 0.85f;
                        float spongeWidth = width * 0.15f;
                        float alpha = std::pow((x - spongeStart) / spongeWidth, 2) * 0.1f;

                        float uInletSqr = 1.5f * (uInlet * uInlet);
                        for (int i = 0; i < 9; i++) {
                            float cu = 3.0f * (cxs[i] * uInlet);
                            float feqInlet = weights[i] * (1.0f + cu + 0.5f * cu * cu - uInletSqr);
                            f[fBase + i] = f[fBase + i] * (1.0f - alpha) + feqInlet * alpha;
                        }
                    }
                }
            }
        }
    }
};



// Fast inline heatmap mapping (Blue -> Green -> Red)
inline uint32_t heatMapColor(float value) {
    value = std::max(0.0f, std::min(1.0f, value));
    uint32_t r = (uint32_t)(std::max(0.0f, std::min(1.0f, 2.0f * value - 1.0f)) * 255.0f);
    uint32_t g = (uint32_t)(std::max(0.0f, std::min(1.0f, 2.0f - 2.0f * std::abs(value - 0.5f))) * 255.0f);
    uint32_t b = (uint32_t)(std::max(0.0f, std::min(1.0f, 1.0f - 2.0f * value)) * 255.0f);
    return 0xFF000000 | (r << 16) | (g << 8) | b;
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

    LBMEngine* engine = reinterpret_cast<LBMEngine*>(ptr);

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
LBMEngine* engine = reinterpret_cast<LBMEngine*>(ptr);

for (int i = 0; i < steps; i++) {
engine->step();
}

void* pixels;
if (AndroidBitmap_lockPixels(env, bitmap, &pixels) < 0) return;
uint32_t* bmpPixels = static_cast<uint32_t*>(pixels);

float maxVel = engine->uInlet * 1.8f;
int totalPixels = engine->width * engine->height;

#pragma omp parallel for schedule(static)
for (int i = 0; i < totalPixels; i++) {
if (engine->obstacles[i]) {
bmpPixels[i] = 0xFFFFFFFF;
} else {
bmpPixels[i] = heatMapColor(engine->velocityMag[i] / maxVel);
}
}
AndroidBitmap_unlockPixels(env, bitmap);
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_resetSimulationNative(
        JNIEnv *env, jobject thiz, jlong ptr) {

    LBMEngine* engine = reinterpret_cast<LBMEngine*>(ptr);;
    engine->reset();
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_latticeboltzmann_NativeLBMEngine_setInletVelocityNative(
        JNIEnv *env, jobject thiz, jlong ptr, jfloat velocity) {

    LBMEngine* engine = reinterpret_cast<LBMEngine*>(ptr);
    // Limit velocity for numerical stability (Mach number should remain low)
    engine->uInletTarget = std::max(0.0f, std::min(0.2f, velocity));
}