# Implementation Plan - Fix UnsatisfiedLinkError for libomp.so

The application is failing with `java.lang.UnsatisfiedLinkError: library "libomp.so" not found`. This is because the OpenMP runtime is not bundled with the APK by default.

I will fix this by linking OpenMP statically into `liblatticeboltzmann.so`. This avoids the need for an external `libomp.so` file and simplifies deployment.

## User Review Required

> [!IMPORTANT]
> I am choosing to link OpenMP statically. This will slightly increase the size of `liblatticeboltzmann.so` but ensures the app runs without needing to manually manage `libomp.so` files in the APK.

## Proposed Changes

### [Component: Native Build]

#### [MODIFY] [CMakeLists.txt](file:///C:/Users/bryce/AndroidStudioProjects/LatticeBoltzmann/app/src/main/cpp/CMakeLists.txt)
- Locate the static `libomp.a` in the NDK.
- Link `latticeboltzmann` against the static library instead of the shared one.

### [Component: Kotlin Implementation]

#### [MODIFY] [NativeLBMEngine.kt](file:///C:/Users/bryce/AndroidStudioProjects/LatticeBoltzmann/app/src/main/java/com/example/latticeboltzmann/NativeLBMEngine.kt)
- (Optional) Ensure `System.loadLibrary("latticeboltzmann")` is handled correctly. (Actually, no changes needed here if static linking works, as the dependency on `libomp.so` will be gone).

## Verification Plan

### Automated Tests
- I will attempt a gradle build to ensure the linking works correctly.
  - `gradlew :app:assembleDebug`

### Manual Verification
- The user should run the app and verify that the `FATAL EXCEPTION: main` with `UnsatisfiedLinkError` is gone.
