package com.example.latticeboltzmann

import android.graphics.Bitmap

class NativeLBMEngine(val width: Int, val height: Int) {
    private var enginePtr: Long = 0

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

    fun getInletVelocity(): Float {
        return if (enginePtr != 0L) getInletVelocityNative(enginePtr) else 0.0f
    }

    fun getDragForce(): Float {
        return if (enginePtr != 0L) getDragForceNative(enginePtr) else 0.0f
    }

    private external fun initEngine(width: Int, height: Int): Long
    private external fun stepAndRenderNative(ptr: Long, bitmap: Bitmap, steps: Int)
    private external fun addObstacleNative(ptr: Long, cx: Int, cy: Int, radius: Int)
    private external fun resetSimulationNative(ptr: Long)
    private external fun setInletVelocityNative(ptr: Long, velocity: Float)
    private external fun getInletVelocityNative(ptr: Long): Float
    private external fun getDragForceNative(ptr: Long): Float
    private external fun destroyEngine(ptr: Long)
}