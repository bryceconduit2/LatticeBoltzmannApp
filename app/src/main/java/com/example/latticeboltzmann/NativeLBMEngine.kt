package com.example.latticeboltzmann

import android.graphics.Bitmap

class NativeLBMEngine(val width: Int, val height: Int) {
    private var enginePtr: Long = 0

    companion object {
        const val VIZ_VELOCITY = 0
        const val VIZ_PRESSURE = 1
        const val VIZ_TOTAL_PRESSURE = 2

        const val BND_PERIODIC = 0
        const val BND_NO_SLIP = 1
        const val BND_FREE_SLIP = 2
    }

    init {
        android.util.Log.d("NativeLBMEngine", "Loading libomp.so...")
        try {
            System.loadLibrary("omp")
            android.util.Log.d("NativeLBMEngine", "libomp.so loaded successfully")
        } catch (e: UnsatisfiedLinkError) {
            android.util.Log.e("NativeLBMEngine", "Failed to load libomp.so: ${e.message}")
        }

        android.util.Log.d("NativeLBMEngine", "Loading liblatticeboltzmann.so...")
        try {
            System.loadLibrary("latticeboltzmann")
            android.util.Log.d("NativeLBMEngine", "liblatticeboltzmann.so loaded successfully")
        } catch (e: UnsatisfiedLinkError) {
            android.util.Log.e("NativeLBMEngine", "Failed to load liblatticeboltzmann.so: ${e.message}")
            throw e
        }
        enginePtr = initEngine(width, height)
    }

    fun stepAndRender(bitmap: Bitmap, steps: Int) {
        stepAndRenderNative(enginePtr, bitmap, steps)
    }

    fun addObstacle(cx: Int, cy: Int, radius: Int) {
        addObstacleNative(enginePtr, cx, cy, radius)
    }

    fun addBoxObstacle(cx: Int, cy: Int, size: Int) {
        if (enginePtr != 0L) {
            addBoxObstacleNative(enginePtr, cx, cy, size)
        }
    }

    fun addNacaAirfoil(cx: Int, cy: Int, chord: Int, m: Float, p: Float, t: Float, angle: Float) {
        if (enginePtr != 0L) {
            addNacaAirfoilNative(enginePtr, cx, cy, chord, m, p, t, angle)
        }
    }

    protected fun finalize() {
        if (enginePtr != 0L) {
            destroyEngine(enginePtr)
            enginePtr = 0L
        }
    }
    // 1. Add the public method for WindTunnelView to call
    fun resetSimulation() {
        if (enginePtr != 0L) {
            resetSimulationNative(enginePtr)
        }
    }

    fun setInletVelocity(velocity: Float) {
        if (enginePtr != 0L) {
            setInletVelocityNative(enginePtr, velocity)
        }
    }

    fun setDensity(density: Float) {
        if (enginePtr != 0L) {
            setDensityNative(enginePtr, density)
        }
    }

    fun setViscosity(viscosity: Float) {
        if (enginePtr != 0L) {
            setViscosityNative(enginePtr, viscosity)
        }
    }

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

    fun getActiveCores(): Int {
        return if (enginePtr != 0L) getActiveCoresNative(enginePtr) else 0
    }

    fun getMaxCores(): Int = getMaxCoresNative()

    fun setNumThreads(n: Int) {
        if (enginePtr != 0L) {
            setNumThreadsNative(enginePtr, n)
        }
    }

    private external fun initEngine(width: Int, height: Int): Long
    private external fun stepAndRenderNative(ptr: Long, bitmap: Bitmap, steps: Int)
    private external fun addObstacleNative(ptr: Long, cx: Int, cy: Int, radius: Int)
    private external fun addBoxObstacleNative(ptr: Long, cx: Int, cy: Int, size: Int)
    private external fun addNacaAirfoilNative(ptr: Long, cx: Int, cy: Int, chord: Int, m: Float, p: Float, t: Float, angle: Float)
    private external fun resetSimulationNative(ptr: Long)
    private external fun setInletVelocityNative(ptr: Long, velocity: Float)
    private external fun setDensityNative(ptr: Long, density: Float)
    private external fun setViscosityNative(ptr: Long, viscosity: Float)
    private external fun setVisualizationModeNative(ptr: Long, mode: Int)
    private external fun setBoundaryModeNative(ptr: Long, mode: Int)
    private external fun getInletVelocityNative(ptr: Long): Float
    private external fun getDragForceNative(ptr: Long): Float
    private external fun getDragCoefficientNative(ptr: Long): Float
    private external fun getInstantDragCoefficientNative(ptr: Long): Float
    private external fun getLiftForceNative(ptr: Long): Float
    private external fun getLiftCoefficientNative(ptr: Long): Float
    private external fun getInstantLiftCoefficientNative(ptr: Long): Float
    private external fun getTotalStepsNative(ptr: Long): Long
    private external fun getDensityNative(ptr: Long): Float
    private external fun getViscosityNative(ptr: Long): Float
    private external fun getDXNative(ptr: Long): Float
    private external fun destroyEngine(ptr: Long)
    private external fun getMaxCoresNative(): Int
    private external fun setNumThreadsNative(ptr: Long, n: Int)
    private external fun getActiveCoresNative(ptr: Long): Int
}