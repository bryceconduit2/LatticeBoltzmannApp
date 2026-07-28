# Implementation Plan - Fix libomp.so UnsatisfiedLinkError

The application is failing at runtime because `libomp.so` (OpenMP library) is not found. This is a common issue when using OpenMP in Android NDK projects, as `libomp.so` is not a system library and must be bundled with the APK.

## User Review Required

> [!IMPORTANT]
> This change modifies how native libraries are linked and loaded. It adds an explicit load call for `libomp` in the Kotlin code to ensure it's available before the main native engine is initialized.

## Proposed Changes

### Build Configuration

#### [MODIFY] [CMakeLists.txt](file:///C:/Users/bryce/AndroidStudioProjects/LatticeBoltzmann/app/src/main/cpp/CMakeLists.txt)
- Update to explicitly find and import `libomp.so` from the NDK.
- Ensure `libomp` is treated as a shared imported library so it gets bundled into the APK.

#### [MODIFY] [NativeLBMEngine.kt](file:///C:/Users/bryce/AndroidStudioProjects/LatticeBoltzmann/app/src/main/java/com/example/latticeboltzmann/NativeLBMEngine.kt)
- Add `System.loadLibrary("omp")` before `System.loadLibrary("latticeboltzmann")`.

## Verification Plan

### Automated Tests
- Build the project using Gradle: `./gradlew assembleDebug`
- Verify that `libomp.so` is present in the generated APK (I will check the build logs or assume success if it builds and runs on a device/emulator).

### Manual Verification
- Deploy the app to an emulator or device and verify that it no longer crashes with `UnsatisfiedLinkError`.
