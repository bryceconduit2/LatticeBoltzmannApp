package com.bc.fluidsandbox

import android.graphics.Bitmap

/**
 * High-performance JNI Bridge for the C++ Lattice Boltzmann Engine.
 * 
 * This class serves as the exclusive communication layer between the Android UI 
 * and the low-level physics kernel. It manages the lifecycle of the native engine
 * pointer and exposes scientific parameters to the Kotlin environment.
 */
class NativeLBMEngine(val width: Int, val height: Int) {
    // Memory address of the C++ LBMEngine object
    private var enginePtr: Long = 0

    companion object {
        // Visualization constants (must match enum in lbm_engine.cpp)
        const val VIZ_VELOCITY = 0
        const val VIZ_PRESSURE = 1
        const val VIZ_TOTAL_PRESSURE = 2

        // Boundary condition constants
        const val BND_PERIODIC = 0
        const val BND_NO_SLIP = 1
        const val BND_FREE_SLIP = 2
        const val BND_OPEN = 3
    }

    init {
        // Load the OpenMP parallel processing library first
        android.util.Log.d("NativeLBMEngine", "Loading libomp.so...")
        try {
            System.loadLibrary("omp")
            android.util.Log.d("NativeLBMEngine", "libomp.so loaded successfully")
        } catch (e: UnsatisfiedLinkError) {
            android.util.Log.e("NativeLBMEngine", "Failed to load libomp.so: ${e.message}")
        }

        // Load the core physics library (Fluid Sandbox Engine)
        android.util.Log.d("NativeLBMEngine", "Loading libfluidsandbox.so...")
        try {
            System.loadLibrary("fluidsandbox")
            android.util.Log.d("NativeLBMEngine", "libfluidsandbox.so loaded successfully")
        } catch (e: UnsatisfiedLinkError) {
            android.util.Log.e("NativeLBMEngine", "Failed to load libfluidsandbox.so: ${e.message}")
            throw e
        }
        
        // Initialize the native engine and store the pointer
        enginePtr = initEngine(width, height)
    }

    /**
     * Executes physics steps and renders the result directly into a Bitmap.
     */
    fun stepAndRender(bitmap: Bitmap, steps: Int, drawBlack: Boolean = true) {
        stepAndRenderNative(enginePtr, bitmap, steps, drawBlack)
    }

    /**
     * Executes physics steps without rendering (used for high-speed background tests).
     */
    fun step(steps: Int) {
        if (enginePtr != 0L) {
            stepNative(enginePtr, steps)
        }
    }

    /**
     * Scientific drawing: Adds a circular obstacle to the grid.
     */
    fun addObstacle(cx: Int, cy: Int, radius: Int) {
        addObstacleNative(enginePtr, cx, cy, radius)
    }

    /**
     * Scientific drawing: Adds a rectangular obstacle to the grid.
     */
    fun addBoxObstacle(cx: Int, cy: Int, size: Int) {
        if (enginePtr != 0L) {
            addBoxObstacleNative(enginePtr, cx, cy, size)
        }
    }

    /**
     * Scientific drawing: Generates a NACA 4-digit airfoil using parametric equations.
     */
    fun addNacaAirfoil(cx: Int, cy: Int, chord: Int, m: Float, p: Float, t: Float, angle: Float) {
        if (enginePtr != 0L) {
            addNacaAirfoilNative(enginePtr, cx, cy, chord, m, p, t, angle)
        }
    }

    /**
     * Cleanup: Ensures the C++ memory is freed when the Kotlin object is garbage collected.
     */
    protected fun finalize() {
        if (enginePtr != 0L) {
            destroyEngine(enginePtr)
            enginePtr = 0L
        }
    }

    /**
     * Resets the entire simulation to a uniform initial flow state.
     */
    fun resetSimulation() {
        if (enginePtr != 0L) {
            resetSimulationNative(enginePtr)
        }
    }

    fun isAerodynamicsValid(): Boolean {
        return if (enginePtr != 0L) isAerodynamicsValidNative(enginePtr) else true
    }

    /**
     * Updates the inlet (wind) velocity in m/s.
     */
    fun setInletVelocity(velocity: Float) {
        if (enginePtr != 0L) {
            setInletVelocityNative(enginePtr, velocity)
        }
    }

    /**
     * Updates the fluid density (kg/m^3).
     */
    fun setDensity(density: Float) {
        if (enginePtr != 0L) {
            setDensityNative(enginePtr, density)
        }
    }

    /**
     * Updates the kinematic viscosity (m^2/s).
     */
    fun setViscosity(viscosity: Float) {
        if (enginePtr != 0L) {
            setViscosityNative(enginePtr, viscosity)
        }
    }

    /**
     * Sets the simulation time step (dt) for math precision control.
     */
    fun setDeltaTime(dt: Float) {
        if (enginePtr != 0L) {
            setDeltaTimeNative(enginePtr, dt)
        }
    }

    // --- Data Retrieval Methods for Telemetry & Graphs ---

    fun getInletVelocity(): Float {
        return if (enginePtr != 0L) getInletVelocityNative(enginePtr) else 0.0f
    }

    fun getDragForce(): Float {
        return if (enginePtr != 0L) getDragForceNative(enginePtr) else 0.0f
    }

    fun getDragCoefficient(): Float {
        return if (enginePtr != 0L) getDragCoefficientNative(enginePtr) else 0.0f
    }

    fun getInstantDragCoefficient(): Float {
        return if (enginePtr != 0L) getInstantDragCoefficientNative(enginePtr) else 0.0f
    }

    fun getLiftForce(): Float {
        return if (enginePtr != 0L) getLiftForceNative(enginePtr) else 0.0f
    }

    fun getLiftCoefficient(): Float {
        return if (enginePtr != 0L) getLiftCoefficientNative(enginePtr) else 0.0f
    }

    fun getInstantLiftCoefficient(): Float {
        return if (enginePtr != 0L) getInstantLiftCoefficientNative(enginePtr) else 0.0f
    }

    fun getTotalSteps(): Long {
        return if (enginePtr != 0L) getTotalStepsNative(enginePtr) else 0L
    }

    fun getDensity(): Float {
        return if (enginePtr != 0L) getDensityNative(enginePtr) else 0.0f
    }

    fun getViscosity(): Float {
        return if (enginePtr != 0L) getViscosityNative(enginePtr) else 0.0f
    }

    fun getDX(): Float {
        return if (enginePtr != 0L) getDXNative(enginePtr) else 0.0f
    }

    fun getHorizontalSpan(): Float {
        return if (enginePtr != 0L) getHorizontalSpanNative(enginePtr) else 0.0f
    }

    fun setVisualizationMode(mode: Int) {
        if (enginePtr != 0L) {
            setVisualizationModeNative(enginePtr, mode)
        }
    }

    fun setBoundaryMode(mode: Int) {
        if (enginePtr != 0L) {
            setBoundaryModeNative(enginePtr, mode)
        }
    }

    fun setColorScheme(scheme: Int) {
        if (enginePtr != 0L) {
            setColorSchemeNative(enginePtr, scheme)
        }
    }

    fun getActiveCores(): Int {
        return if (enginePtr != 0L) getActiveCoresNative(enginePtr) else 0
    }

    fun getMaxCores(): Int = getMaxCoresNative()

    fun setNumThreads(n: Int) {
        if (enginePtr != 0L) {
            setNumThreadsNative(enginePtr, n)
        }
    }

    // --- Native JNI Method Declarations ---

    private external fun initEngine(width: Int, height: Int): Long
    private external fun stepNative(ptr: Long, steps: Int)
    private external fun stepAndRenderNative(ptr: Long, bitmap: Bitmap, steps: Int, drawBlack: Boolean)
    private external fun addObstacleNative(ptr: Long, cx: Int, cy: Int, radius: Int)
    private external fun addBoxObstacleNative(ptr: Long, cx: Int, cy: Int, size: Int)
    private external fun addNacaAirfoilNative(ptr: Long, cx: Int, cy: Int, chord: Int, m: Float, p: Float, t: Float, angle: Float)
    private external fun resetSimulationNative(ptr: Long)
    private external fun setInletVelocityNative(ptr: Long, velocity: Float)
    private external fun setDensityNative(ptr: Long, density: Float)
    private external fun setViscosityNative(ptr: Long, viscosity: Float)
    private external fun setDeltaTimeNative(ptr: Long, dt: Float)
    private external fun setVisualizationModeNative(ptr: Long, mode: Int)
    private external fun setBoundaryModeNative(ptr: Long, mode: Int)
    private external fun setColorSchemeNative(ptr: Long, scheme: Int)
    private external fun getInletVelocityNative(ptr: Long): Float
    private external fun getDragForceNative(ptr: Long): Float
    private external fun getDragCoefficientNative(ptr: Long): Float
    private external fun getInstantDragCoefficientNative(ptr: Long): Float
    private external fun getLiftForceNative(ptr: Long): Float
    private external fun getLiftCoefficientNative(ptr: Long): Float
    private external fun getInstantLiftCoefficientNative(ptr: Long): Float
    private external fun isAerodynamicsValidNative(ptr: Long): Boolean
    private external fun getTotalStepsNative(ptr: Long): Long
    private external fun getDensityNative(ptr: Long): Float
    private external fun getViscosityNative(ptr: Long): Float
    private external fun getDXNative(ptr: Long): Float
    private external fun getHorizontalSpanNative(ptr: Long): Float
    private external fun destroyEngine(ptr: Long)
    private external fun getMaxCoresNative(): Int
    private external fun setNumThreadsNative(ptr: Long, n: Int)
    private external fun getActiveCoresNative(ptr: Long): Int
}
