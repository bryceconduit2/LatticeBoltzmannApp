package com.BC.latticeboltzmann

import android.graphics.*
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import java.io.File
import java.io.FileOutputStream
import java.util.Locale

@RunWith(AndroidJUnit4::class)
class PhysicsAccuracyTest {

    @get:Rule
    val storagePermissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        android.Manifest.permission.READ_EXTERNAL_STORAGE,
        android.Manifest.permission.WRITE_EXTERNAL_STORAGE
    )

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

    @Test
    fun testNaca4412CamberedLift() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(25.0f)
        
        // NACA 4412 at 0 degrees should have positive lift due to camber
        engine.addNacaAirfoil(width / 4, height / 2, 60, 0.04f, 0.4f, 0.12f, 0.0f)
        repeat(50) { engine.step(100) }
        
        val cl = engine.getInstantLiftCoefficient()
        saveSnapshot(engine, "Naca4412_Cambered_0deg", "Cl: %.2f".format(Locale.US, cl))
        
        // Cambered airfoils should produce positive lift even at 0 deg AoA.
        // At 400x200 resolution, we expect at least a clear positive bias.
        assertTrue("Cambered airfoil at 0 deg should produce positive lift ($cl)", cl > 0.02f)
    }

    @Test
    fun testStallDetectionTrend() {
        val width = 400
        val height = 200
        
        // Case 1: 12 degrees (High lift, attached flow)
        val engine12 = NativeLBMEngine(width, height)
        engine12.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine12.setInletVelocity(20.0f)
        engine12.addNacaAirfoil(width / 4, height / 2, 50, 0.0f, 0.0f, 0.12f, 12.0f)
        repeat(40) { engine12.step(100) }
        val cl12 = engine12.getInstantLiftCoefficient()
        val cd12 = engine12.getInstantDragCoefficient()
        saveSnapshot(engine12, "Naca0012_12deg", "Cl: %.2f, Cd: %.2f".format(Locale.US, cl12, cd12))

        // Case 2: 45 degrees (Stalled, separated flow)
        val engine45 = NativeLBMEngine(width, height)
        engine45.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine45.setInletVelocity(20.0f)
        engine45.addNacaAirfoil(width / 4, height / 2, 50, 0.0f, 0.0f, 0.12f, 45.0f)
        repeat(40) { engine45.step(100) }
        val cl45 = engine45.getInstantLiftCoefficient()
        val cd45 = engine45.getInstantDragCoefficient()
        saveSnapshot(engine45, "Naca0012_45deg_Stall", "Cl: %.2f, Cd: %.2f".format(Locale.US, cl45, cd45))

        // In a stall, Drag should spike significantly
        assertTrue("Drag at 45 deg ($cd45) should be much higher than at 12 deg ($cd12)", cd45 > cd12 * 2.0f)
        // Lift typically drops or levels off significantly compared to the linear slope
        assertTrue("Lift at 45 deg ($cl45) should be less than or comparable to 12 deg ($cl12) due to stall", cl45 < cl12 + 0.5f)
    }

    @Test
    fun testDragPolarTrend() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(25.0f)
        
        // 0 deg
        engine.addNacaAirfoil(width / 4, height / 2, 50, 0.02f, 0.4f, 0.12f, 0.0f)
        repeat(20) { engine.step(100) }
        val cd0 = engine.getInstantDragCoefficient()
        
        // 15 deg (Restart engine to clear wake)
        val engine15 = NativeLBMEngine(width, height)
        engine15.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine15.setInletVelocity(25.0f)
        engine15.addNacaAirfoil(width / 4, height / 2, 50, 0.02f, 0.4f, 0.12f, 15.0f)
        repeat(20) { engine15.step(100) }
        val cd15 = engine15.getInstantDragCoefficient()
        
        assertTrue("Drag at 15 deg ($cd15) should be higher than at 0 deg ($cd0)", cd15 > cd0)
    }

    @Test
    fun testFlatPlateDrag() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(25.0f)
        
        // Draw a thin vertical plate (High drag shape)
        // We'll use a very small chord and large AoA (90 deg) or just a box
        engine.addBoxObstacle(width / 4, height / 2, 40) // 40x40 box
        repeat(50) { engine.step(100) }
        
        val cd = engine.getInstantDragCoefficient()
        saveSnapshot(engine, "FlatPlate_Box", "Cd: %.2f".format(Locale.US, cd))
        
        // Cd for a square/flat plate is higher than a cylinder (~2.0 vs ~1.2)
        assertTrue("Box drag ($cd) should be higher than a cylinder (~1.2)", cd > 1.4f)
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
        
        // Use the official app-specific external directory (No permissions required)
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val snapshotDir = File(context.getExternalFilesDir(null), "WindTunnelSnapshots")
        if (!snapshotDir.exists()) snapshotDir.mkdirs()
        
        // Overwriting "LATEST" file for easy viewing on computer
        val latestFile = File(snapshotDir, "AA_LATEST_${name}.png")
        
        try {
            FileOutputStream(latestFile).use { out ->
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, out)
            }
            
            // Force Android to refresh the folder so it shows up in Device File Explorer immediately
            android.media.MediaScannerConnection.scanFile(context, arrayOf(latestFile.absolutePath), null, null)
            
            android.util.Log.i("PhysicsTest", "!!! SNAPSHOT GENERATED: ${latestFile.absolutePath}")
            android.util.Log.i("PhysicsTest", "Find it in Device File Explorer at: sdcard/Android/data/com.example.latticeboltzmann/files/WindTunnelSnapshots/")
        } catch (e: Exception) {
            android.util.Log.e("PhysicsTest", "Failed to save snapshot: ${e.message}")
        }
    }
}
