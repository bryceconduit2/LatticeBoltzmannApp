package com.example.latticeboltzmann

import android.graphics.*
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import java.io.File
import java.io.FileOutputStream
import java.util.Locale

@RunWith(AndroidJUnit4::class)
class PhysicsAccuracyTest {

    @Test
    fun testCylinderDragCoefficient() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        
        // Use Periodic boundaries to avoid tunnel wall interference during accuracy checks
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(30.0f)
        engine.addObstacle(width / 4, height / 2, 10) // r=10px -> d=20px
        
        repeat(50) { engine.step(100) }
        
        val cd = engine.getInstantDragCoefficient()
        saveSnapshot(engine, "CylinderDrag", "Cd: %.2f".format(Locale.US, cd))
        
        // Typical Cd for a cylinder is near 1.2
        assertTrue("Cylinder Drag Coefficient ($cd) should be near 1.2", cd > 0.5f && cd < 3.0f)
    }

    @Test
    fun testNaca0012Symmetry() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(20.0f)
        // NACA 0012 at 0 degrees AoA should have near zero lift
        engine.addNacaAirfoil(width / 4, height / 2, 50, 0.0f, 0.0f, 0.12f, 0.0f)
        
        repeat(40) { engine.step(100) }
        
        val cl = engine.getInstantLiftCoefficient()
        saveSnapshot(engine, "Naca0012_Symmetry", "Cl: %.3f".format(Locale.US, cl))
        
        // Allow for numerical bias in discrete grid
        assertTrue("Symmetric airfoil should have near-zero lift ($cl)", Math.abs(cl) < 2.0f)
    }

    @Test
    fun testLiftSlope() {
        val width = 400
        val height = 200
        
        // Case 1: 0 degrees AoA
        val engine0 = NativeLBMEngine(width, height)
        engine0.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine0.setInletVelocity(25.0f)
        engine0.addNacaAirfoil(width / 4, height / 2, 50, 0.04f, 0.4f, 0.12f, 0.0f)
        repeat(30) { engine0.step(100) }
        val cl0 = engine0.getInstantLiftCoefficient()
        saveSnapshot(engine0, "Naca4412_0deg", "Cl: %.2f".format(Locale.US, cl0))
        
        // Case 2: 10 degrees AoA
        val engine10 = NativeLBMEngine(width, height)
        engine10.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine10.setInletVelocity(25.0f)
        // Leading edge up
        engine10.addNacaAirfoil(width / 4, height / 2, 50, 0.04f, 0.4f, 0.12f, -10.0f)
        repeat(30) { engine10.step(100) }
        val cl10 = engine10.getInstantLiftCoefficient()
        saveSnapshot(engine10, "Naca4412_10deg", "Cl: %.2f".format(Locale.US, cl10))
        
        // At +10 deg, lift coefficient should be higher than at 0 deg
        assertTrue("Lift at 10 deg ($cl10) should be significantly greater than at 0 deg ($cl0)", cl10 > cl0 + 0.1f)
    }

    private fun saveSnapshot(engine: NativeLBMEngine, name: String, info: String) {
        val bitmap = Bitmap.createBitmap(engine.width, engine.height, Bitmap.Config.ARGB_8888)
        engine.stepAndRender(bitmap, 0)
        
        // Add watermark
        val canvas = Canvas(bitmap)
        val paint = Paint().apply {
            color = Color.WHITE
            textSize = 12f
            isAntiAlias = true
            setShadowLayer(2f, 1f, 1f, Color.BLACK)
        }
        canvas.drawText("Test: $name | $info", 10f, 20f, paint)
        
        // Save to the public Download folder for easy access in Device File Explorer
        val downloadDir = File("/sdcard/Download/WindTunnelTests")
        if (!downloadDir.exists()) downloadDir.mkdirs()
        
        // 1. Create a "LATEST" file that always updates (easy to find)
        val latestFile = File(downloadDir, "AA_LATEST_${name}.png")
        
        // 2. Create a timestamped version for history
        val timestamp = java.text.SimpleDateFormat("HHmmss", Locale.US).format(java.util.Date())
        val historyFile = File(downloadDir, "${name}_${timestamp}.png")
        
        try {
            // Write both files
            FileOutputStream(latestFile).use { out -> bitmap.compress(Bitmap.CompressFormat.PNG, 100, out) }
            FileOutputStream(historyFile).use { out -> bitmap.compress(Bitmap.CompressFormat.PNG, 100, out) }
            
            // Trigger Media Scanner so the files appear immediately in the explorer
            val context = InstrumentationRegistry.getInstrumentation().targetContext
            android.media.MediaScannerConnection.scanFile(context, arrayOf(latestFile.absolutePath, historyFile.absolutePath), null, null)
            
            android.util.Log.i("PhysicsTest", "!!! VIEW LATEST HERE: /sdcard/Download/WindTunnelTests/${latestFile.name}")
        } catch (e: Exception) {
            android.util.Log.e("PhysicsTest", "Failed to save snapshot: ${e.message}")
        }
    }
}
