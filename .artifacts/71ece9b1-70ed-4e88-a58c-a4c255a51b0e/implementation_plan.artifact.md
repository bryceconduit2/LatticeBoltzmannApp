# Implementation Plan - Seamless Row-Based Adaptive Optimization

This plan optimizes the simulation speed by identifying uniform (laminar) flow rows and skipping their collision math, without creating the checkerboard "interlacing" artifacts.

## User Review Required

> [!NOTE]
> **Turbulence Awareness**: Information in fluid dynamics propagates. To ensure turbulence correctly "wakes up" the optimized regions, I will implement a **Safety Buffer**. Rows adjacent to turbulent or obstacle regions will always be calculated with full physics, ensuring that vortices aren't cut off or ignored as they travel through the tunnel.

## Proposed Changes

### [C++ Engine]

#### [MODIFY] [lbm_engine.cpp](file:///C:/Users/bryce/AndroidStudioProjects/LatticeBoltzmannApp/app/src/main/cpp/lbm_engine.cpp)
- **Remove Checkerboard**: Delete the `(x + y + totalSteps) % 2` logic to restore a smooth visual field.
- **Complexity Analysis**:
    - For each row, calculate a `rowComplexity` score based on the non-equilibrium energy of all cells in that row.
    - **Fast Path (Laminar)**: If a row is purely fluid and has near-zero complexity, use a simplified "Stream-Only" kernel.
    - **Full Path (Turbulent)**: If any cell in the row (or its immediate neighbor rows) is complex, perform the full Maxwell-Boltzmann collision.
- **SIMD Rendering**: Restored full 1:1 resolution rendering in Performance Mode, but using a single-pass optimized loop.

### [Android App]
- No UI changes required.

## Verification Plan

### Manual Verification
- **Interlacing Check**: Verify that the fluid looks perfectly smooth (no checkerboard dots) in Performance mode.
- **Turbulence Test**: Draw a shape. Verify that the turbulent wake correctly "spreads" and forces full calculations in the affected downstream rows.
- **FPS Check**: Compare FPS on the Fire Tablet 7. Expecting a significant gain over "Higher Accuracy" while remaining visually clean.
