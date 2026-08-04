package com.bc.fluidsandbox

import android.graphics.*
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.rule.GrantPermissionRule
import com.bc.fluidsandbox.NativeLBMEngine
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import java.io.File
import java.io.FileOutputStream
import java.util.Locale

/**
 * Rigorous Aerodynamic Validation Suite for Fluid Sandbox.
 * 
 * These tests run the C++ physics engine in a headless "lab" environment
 * to verify that the math matches theoretical and empirical wind tunnel data.
 */
@RunWith(AndroidJUnit4::class)
class PhysicsAccuracyTest {

    @get:Rule
    val storagePermissionRule: GrantPermissionRule = GrantPermissionRule.grant(
        android.Manifest.permission.READ_EXTERNAL_STORAGE,
        android.Manifest.permission.WRITE_EXTERNAL_STORAGE
    )

    /**
     * BENCHMARK: Flow around a circular cylinder.
     * Checks if the Drag Coefficient (Cd) matches standard experimental data (~1.2).
     */
    @Test
    fun testCylinderDragCoefficient() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(30.0f)
        engine.addObstacle(width / 4, height / 2, 10) 
        
        repeat(150) { engine.step(100) }
        
        val cd = engine.getInstantDragCoefficient()
        saveSnapshot(engine, "CylinderDrag", "Cd: %.2f".format(Locale.US, cd))
        
        assertTrue("Cylinder Cd ($cd) should be near 1.2", cd >= 0.8f && cd <= 2.5f)
    }

    /**
     * SYMMETRY TEST: Symmetric NACA 0012 airfoil at 0 degrees.
     * Verifies that the engine produces near-zero lift when the geometry is balanced.
     */
    @Test
    fun testNaca0012Symmetry() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(20.0f)
        engine.addNacaAirfoil(width / 4, height / 2, 50, 0.0f, 0.0f, 0.12f, 0.0f)
        
        repeat(100) { engine.step(100) }
        
        val cl = engine.getInstantLiftCoefficient()
        saveSnapshot(engine, "Naca0012_Symmetry", "Cl: %.3f".format(Locale.US, cl))
        
        assertTrue("Symmetric airfoil lift ($cl) should be near-zero", Math.abs(cl) < 0.2f)
    }

    /**
     * LIFT SLOPE TEST: Checks if increasing Angle of Attack (AoA) increases lift.
     * Verifies the primary "Flying" mechanism of the simulation.
     */
    @Test
    fun testLiftSlope() {
        val width = 400
        val height = 200
        
        // Measure lift at 0 degrees
        val engine0 = NativeLBMEngine(width, height)
        engine0.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine0.setInletVelocity(25.0f)
        engine0.addNacaAirfoil(width / 4, height / 2, 50, 0.04f, 0.4f, 0.12f, 0.0f)
        repeat(100) { engine0.step(100) }
        val cl0 = engine0.getInstantLiftCoefficient()
        
        // Measure lift at 10 degrees (Nose Up)
        val engine10 = NativeLBMEngine(width, height)
        engine10.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine10.setInletVelocity(25.0f)
        engine10.addNacaAirfoil(width / 4, height / 2, 50, 0.04f, 0.4f, 0.12f, -10.0f)
        repeat(100) { engine10.step(100) }
        val cl10 = engine10.getInstantLiftCoefficient()
        
        assertTrue("Lift at Nose-Up 10 deg ($cl10) should be greater than at 0 deg ($cl0)", cl10 > cl0 + 0.2f)
    }

    /**
     * CAMBER TEST: Highly curved NACA 4412 profile at 0 degrees.
     * Verifies that "Camber" (curvature) creates lift even without tilt.
     */
    @Test
    fun testNaca4412CamberedLift() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(25.0f)
        
        engine.addNacaAirfoil(width / 4, height / 2, 60, 0.04f, 0.4f, 0.12f, 0.0f)
        repeat(150) { engine.step(100) }
        
        val cl = engine.getInstantLiftCoefficient()
        saveSnapshot(engine, "Naca4412_Cambered_0deg", "Cl: %.2f".format(Locale.US, cl))
        
        assertTrue("Cambered airfoil at 0 deg should produce positive lift ($cl)", cl > 0.01f)
    }

    /**
     * STALL TEST: Verifies that drag spikes at extreme angles (45 deg).
     * Simulates the "Breakdown" of aerodynamic lift and massive wake turbulence.
     */
    @Test
    fun testStallDetectionTrend() {
        val width = 400
        val height = 200
        
        // Normal flight (12 deg)
        val engine12 = NativeLBMEngine(width, height)
        engine12.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine12.setInletVelocity(20.0f)
        engine12.addNacaAirfoil(width / 4, height / 2, 50, 0.0f, 0.0f, 0.12f, -12.0f)
        repeat(150) { engine12.step(100) }
        val cd12 = engine12.getInstantDragCoefficient()

        // Stalled flight (45 deg)
        val engine45 = NativeLBMEngine(width, height)
        engine45.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine45.setInletVelocity(20.0f)
        engine45.addNacaAirfoil(width / 4, height / 2, 50, 0.0f, 0.0f, 0.12f, -45.0f)
        repeat(150) { engine45.step(100) }
        val cd45 = engine45.getInstantDragCoefficient()

        assertTrue("Drag at 45 deg ($cd45) should be much higher than at 12 deg ($cd12)", cd45 > cd12 * 2.0f)
    }

    /**
     * POLAR TREND: Checks if drag increases as tilt increases.
     * Verifies that more aggressive maneuvers create more "air resistance."
     */
    @Test
    fun testDragPolarTrend() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(25.0f)
        
        engine.addNacaAirfoil(width / 4, height / 2, 50, 0.02f, 0.4f, 0.12f, 0.0f)
        repeat(100) { engine.step(100) }
        val cd0 = engine.getInstantDragCoefficient()
        
        val engine15 = NativeLBMEngine(width, height)
        engine15.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine15.setInletVelocity(25.0f)
        engine15.addNacaAirfoil(width / 4, height / 2, 50, 0.02f, 0.4f, 0.12f, -15.0f)
        repeat(100) { engine15.step(100) }
        val cd15 = engine15.getInstantDragCoefficient()
        
        assertTrue("Drag at 15 deg ($cd15) should be higher than at 0 deg ($cd0)", cd15 > cd0 + 0.05f)
    }

    /**
     * DRAG COMPARISON: Square box vs Cylinder.
     * Verifies that "Blunt" shapes create more drag than "Rounded" ones.
     */
    @Test
    fun testFlatPlateDrag() {
        val width = 400
        val height = 200
        val engine = NativeLBMEngine(width, height)
        engine.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine.setInletVelocity(25.0f)
        
        engine.addBoxObstacle(width / 4, height / 2, 40) 
        repeat(150) { engine.step(100) }
        
        val cd = engine.getInstantDragCoefficient()
        saveSnapshot(engine, "FlatPlate_Box", "Cd: %.2f".format(Locale.US, cd))
        
        assertTrue("Box drag ($cd) should be higher than a cylinder (~1.2)", cd > 1.4f)
    }

    /**
     * AERODYNAMIC PRECISION: Verify the "Zero Lift Angle."
     * Cambered wings need to be tilted DOWN (Nose down) to produce zero lift.
     */
    @Test
    fun testNaca2412ZeroLiftAngle() {
        val width = 400
        val height = 200
        
        val engine0 = NativeLBMEngine(width, height)
        engine0.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine0.setInletVelocity(25.0f)
        engine0.addNacaAirfoil(width / 4, height / 2, 60, 0.02f, 0.4f, 0.12f, 0.0f)
        repeat(150) { engine0.step(100) }
        val cl0 = engine0.getInstantLiftCoefficient()

        val engineP2 = NativeLBMEngine(width, height)
        engineP2.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engineP2.setInletVelocity(25.0f)
        engineP2.addNacaAirfoil(width / 4, height / 2, 60, 0.02f, 0.4f, 0.12f, 2.0f)
        repeat(150) { engineP2.step(100) }
        val clP2 = engineP2.getInstantLiftCoefficient()

        assertTrue("NACA 2412 at 0 deg ($cl0) should have more lift than at nose-down 2 deg ($clP2)", cl0 > clP2 + 0.05f)
    }

    /**
     * FORM DRAG TEST: Thicker wings should produce more air resistance.
     * Compares a 12% thickness wing vs a 21% thickness wing.
     */
    @Test
    fun testThicknessDragImpact() {
        val width = 400
        val height = 200
        
        val engine12 = NativeLBMEngine(width, height)
        engine12.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine12.setInletVelocity(30.0f)
        engine12.addNacaAirfoil(width / 4, height / 2, 60, 0.0f, 0.0f, 0.12f, 0.0f)
        repeat(150) { engine12.step(100) }
        val cd12 = engine12.getInstantDragCoefficient()

        val engine21 = NativeLBMEngine(width, height)
        engine21.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engine21.setInletVelocity(30.0f)
        engine21.addNacaAirfoil(width / 4, height / 2, 60, 0.0f, 0.0f, 0.21f, 0.0f)
        repeat(150) { engine21.step(100) }
        val cd21 = engine21.getInstantDragCoefficient()

        assertTrue("Thicker airfoil (0021: $cd21) should have more drag than thinner (0012: $cd12)", cd21 > cd12 + 0.01f)
    }

    /**
     * LINEARITY CHECK: Verifies that lift increases predictably with angle.
     * Proves the math stays stable across the normal flight range.
     */
    @Test
    fun testLiftLinearity() {
        val width = 400
        val height = 200
        
        fun getClAt(alpha: Float): Float {
            val e = NativeLBMEngine(width, height)
            e.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
            e.setInletVelocity(25.0f)
            e.addNacaAirfoil(width / 4, height / 2, 60, 0.0f, 0.0f, 0.12f, -alpha)
            repeat(150) { e.step(100) }
            return e.getInstantLiftCoefficient()
        }

        val cl4 = getClAt(4.0f)
        val cl8 = getClAt(8.0f)
        val cl12 = getClAt(12.0f)

        val diff1 = cl8 - cl4
        val diff2 = cl12 - cl8

        assertTrue("Lift slope should be roughly linear: $diff1 vs $diff2", Math.abs(diff1 - diff2) < 0.2f)
    }

    /**
     * STALL DELAY: Confirms that curved wings stay efficient at high angles.
     * Highly curved wings (4412) should outperform straight wings at 12 degrees.
     */
    @Test
    fun testCamberedStallDelay() {
        val width = 400
        val height = 200
        val alpha = -12.0f 
        
        val engineSym = NativeLBMEngine(width, height)
        engineSym.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engineSym.setInletVelocity(20.0f)
        engineSym.addNacaAirfoil(width / 4, height / 2, 60, 0.0f, 0.0f, 0.12f, alpha)
        repeat(150) { engineSym.step(100) }
        val clSym = engineSym.getInstantLiftCoefficient()

        val engineCam = NativeLBMEngine(width, height)
        engineCam.setBoundaryMode(NativeLBMEngine.BND_PERIODIC)
        engineCam.setInletVelocity(20.0f)
        engineCam.addNacaAirfoil(width / 4, height / 2, 60, 0.04f, 0.4f, 0.12f, alpha)
        repeat(150) { engineCam.step(100) }
        val clCam = engineCam.getInstantLiftCoefficient()

        assertTrue("Cambered airfoil ($clCam) should have more lift than symmetric ($clSym) at 12 deg alpha", clCam > clSym + 0.1f)
    }

    /**
     * EXPORT UTILITY: Generates a visual snapshot of a test case for laboratory review.
     */
    private fun saveSnapshot(engine: NativeLBMEngine, name: String, info: String) {
        val bitmap = Bitmap.createBitmap(engine.width, engine.height, Bitmap.Config.ARGB_8888)
        engine.stepAndRender(bitmap, 0)
        
        val canvas = Canvas(bitmap)
        val paint = Paint().apply {
            color = Color.WHITE
            textSize = 12f
            isAntiAlias = true
            setShadowLayer(2f, 1f, 1f, Color.BLACK)
        }
        canvas.drawText("Test: $name | $info", 10f, 20f, paint)
        
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val snapshotDir = File(context.getExternalFilesDir(null), "FluidSandboxSnapshots")
        if (!snapshotDir.exists()) snapshotDir.mkdirs()
        
        val latestFile = File(snapshotDir, "AA_LATEST_${name}.png")
        
        try {
            FileOutputStream(latestFile).use { out ->
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, out)
            }
            android.media.MediaScannerConnection.scanFile(context, arrayOf(latestFile.absolutePath), null, null)
            android.util.Log.i("PhysicsTest", "!!! SNAPSHOT GENERATED: ${latestFile.absolutePath}")
            android.util.Log.i("PhysicsTest", "Find it in Device File Explorer at: sdcard/Android/data/com.bc.fluidsandbox/files/FluidSandboxSnapshots/")
        } catch (e: Exception) {
            android.util.Log.e("PhysicsTest", "Failed to save snapshot: ${e.message}")
        }
    }
}
