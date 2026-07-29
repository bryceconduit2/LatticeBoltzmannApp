package com.example.latticeboltzmann

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.SurfaceHolder
import android.view.SurfaceView
import java.util.Locale
import kotlin.math.hypot

class WindTunnelView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : SurfaceView(context, attrs), SurfaceHolder.Callback, Runnable {
    enum class BrushShape { CIRCLE, SQUARE }
    private data class ReinitParams(val w: Int, val h: Int)
    
    // High resolution supported natively by C++ OpenMP
    private var simWidth = 400
    private var simHeight = 200
    private var engine = NativeLBMEngine(simWidth, simHeight)
    @Volatile private var pendingReinit: ReinitParams? = null

    private var thread: Thread? = null
    @Volatile private var running = false

    // --- NEW: Variables to track touch state across threads ---
    @Volatile private var isHolding = false
    @Volatile private var touchSimX = 0
    @Volatile private var touchSimY = 0
    private var lastTouchSimX = -1
    private var lastTouchSimY = -1
    private var currentRadius = 8
    private val maxRadius = 60 // Stop growing so it doesn't block the whole tunnel
    
    var brushShape = BrushShape.CIRCLE
    var showDetailedTelemetry = true
    var baseBrushSize = 8
    var visualizationMode = NativeLBMEngine.VIZ_VELOCITY
    var useAbsolutePressure = false

    // Telemetry
    private var lastTime = System.nanoTime()
    private var fps = 0.0
    private var sps = 0.0
    private val stepsPerFrame = 20

    private var bitmap = Bitmap.createBitmap(simWidth, simHeight, Bitmap.Config.ARGB_8888)
    private val paint = Paint().apply { isFilterBitmap = false } // Keep pixels sharp
    
    // Parameter Persistence
    private var currentPhysVelocity = 30.0f
    private var currentPhysDensity = 1.225f
    private var currentPhysViscosity = 1.5e-5f

    private val textPaint = Paint().apply {
        color = Color.WHITE
        textSize = 40f
        isAntiAlias = true
        setShadowLayer(3f, 1f, 1f, Color.BLACK)
    }

    private val previewPaint = Paint().apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = 3f
        isAntiAlias = true
    }

    private val scalePaint = Paint().apply {
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    private val scaleTextPaint = Paint().apply {
        color = Color.WHITE
        textSize = 30f
        isAntiAlias = true
        textAlign = Paint.Align.CENTER
        setShadowLayer(2f, 1f, 1f, Color.BLACK)
    }

    private val scaleColors = intArrayOf(
        Color.BLUE, Color.CYAN, Color.GREEN, Color.YELLOW, Color.RED
    )
    private val scalePositions = floatArrayOf(
        0.0f, 0.25f, 0.5f, 0.75f, 1.0f
    )

    init {
        holder.addCallback(this)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val scaleX = width.toFloat() / simWidth
        val scaleY = height.toFloat() / simHeight

        when (event.action) {
            MotionEvent.ACTION_DOWN -> {
                performClick()
                isHolding = true
                currentRadius = baseBrushSize // Reset to starting size on a new tap
                touchSimX = (event.x / scaleX).toInt()
                touchSimY = (event.y / scaleY).toInt()
                lastTouchSimX = touchSimX
                lastTouchSimY = touchSimY
                return true
            }
            MotionEvent.ACTION_MOVE -> {
                // Update coordinates if they drag their finger while holding
                touchSimX = (event.x / scaleX).toInt()
                touchSimY = (event.y / scaleY).toInt()
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                isHolding = false // Stop growing when finger is lifted
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    override fun performClick(): Boolean {
        super.performClick()
        return true
    }

    override fun run() {
        while (running) {
            // Check for thread-safe re-initialization
            pendingReinit?.let { params ->
                simWidth = params.w
                simHeight = params.h
                bitmap = Bitmap.createBitmap(simWidth, simHeight, Bitmap.Config.ARGB_8888)
                engine = NativeLBMEngine(simWidth, simHeight)
                
                // Re-apply persisted settings
                engine.setInletVelocity(currentPhysVelocity)
                engine.setDensity(currentPhysDensity)
                engine.setViscosity(currentPhysViscosity)
                engine.setVisualizationMode(visualizationMode)
                
                pendingReinit = null
            }

            if (isHolding) {
                val dx = (touchSimX - lastTouchSimX).toFloat()
                val dy = (touchSimY - lastTouchSimY).toFloat()
                val dist = hypot(dx, dy)

                if (dist > 2.0) {
                    // Moved significantly: Act as a brush
                    currentRadius = baseBrushSize
                    
                    // Linear interpolation to fill gaps
                    val steps = Math.ceil(dist / (baseBrushSize / 2.0)).toInt().coerceAtLeast(1)
                    for (i in 1..steps) {
                        val t = i.toFloat() / steps
                        val px = (lastTouchSimX + dx * t).toInt()
                        val py = (lastTouchSimY + dy * t).toInt()
                        addObstacleToEngine(px, py, currentRadius)
                    }
                } else {
                    // Stationary: Grow the current point faster
                    if (currentRadius < maxRadius) {
                        currentRadius += 2 // Increased growth speed
                        if (currentRadius > maxRadius) currentRadius = maxRadius
                    }
                    addObstacleToEngine(touchSimX, touchSimY, currentRadius)
                }
                
                lastTouchSimX = touchSimX
                lastTouchSimY = touchSimY
            }

            val now = System.nanoTime()
            val dtNano = now - lastTime
            if (dtNano > 0) {
                val currentFps = 1_000_000_000.0 / dtNano
                fps = fps * 0.9 + currentFps * 0.1 // Smooth FPS
                sps = fps * stepsPerFrame
            }
            lastTime = now

            // Execute physics steps natively before rendering
            engine.stepAndRender(bitmap, stepsPerFrame)

            val canvas = holder.lockCanvas()
            if (canvas != null) {
                canvas.drawBitmap(bitmap, null, Rect(0, 0, width, height), paint)
                
                // Draw a visual preview of the growing shape
                if (isHolding) {
                    val scaleX = width.toFloat() / simWidth
                    val scaleY = height.toFloat() / simHeight
                    if (brushShape == BrushShape.CIRCLE) {
                        canvas.drawCircle(
                            touchSimX * scaleX, 
                            touchSimY * scaleY, 
                            currentRadius * scaleX, 
                            previewPaint
                        )
                    } else {
                        val r = currentRadius * scaleX
                        val cx = touchSimX * scaleX
                        val cy = touchSimY * scaleY
                        canvas.drawRect(cx - r, cy - r, cx + r, cy + r, previewPaint)
                    }
                }

                val currentVel = engine.getInletVelocity()
                val dragN = engine.getDragForce()
                val cd = engine.getDragCoefficient()
                
                canvas.drawText(String.format(Locale.US, "Velocity: %.1f m/s", currentVel), 30f, 80f, textPaint)
                canvas.drawText(String.format(Locale.US, "Drag: %.2f N | Cd: %.2f", dragN, cd), 30f, 140f, textPaint)
                
                if (showDetailedTelemetry) {
                    val dx = engine.getDX()
                    val physW = simWidth * dx
                    val physH = simHeight * dx
                    val density = engine.getDensity()
                    val viscosity = engine.getViscosity()

                    canvas.drawText(String.format(Locale.US, "Tunnel: %.1fm x %.1fm (1.0m Depth)", physW, physH), 30f, 200f, textPaint)
                    canvas.drawText(String.format(Locale.US, "Density: %.2f kg/m³ | Visc: %.1e m²/s", density, viscosity), 30f, 260f, textPaint)
                    canvas.drawText(String.format(Locale.US, "Performance: %.0f FPS | %.0f Steps/s", fps, sps), 30f, 320f, textPaint)
                }

                drawScaleBar(canvas)
                
                holder.unlockCanvasAndPost(canvas)
            }
        }
    }

    private fun addObstacleToEngine(x: Int, y: Int, radius: Int) {
        if (brushShape == BrushShape.CIRCLE) {
            engine.addObstacle(x, y, radius)
        } else {
            engine.addBoxObstacle(x, y, radius * 2)
        }
    }

    private fun drawScaleBar(canvas: Canvas) {
        val barWidth = 600f // Increased from 400f
        val barHeight = 40f // Increased from 30f
        val margin = 50f
        
        val left = width - barWidth - margin
        val right = width - margin
        val bottom = height - margin - 20f // Added some bottom padding for text clearance
        val top = bottom - barHeight

        // Create gradient if not already set or if width changed
        if (scalePaint.shader == null) {
            scalePaint.shader = LinearGradient(
                left, 0f, right, 0f,
                scaleColors, scalePositions, Shader.TileMode.CLAMP
            )
        }

        // Draw the color bar
        canvas.drawRect(left, top, right, bottom, scalePaint)
        
        // Draw a thin border
        previewPaint.strokeWidth = 2f
        canvas.drawRect(left, top, right, bottom, previewPaint)

        // Draw Labels
        if (visualizationMode == NativeLBMEngine.VIZ_VELOCITY) {
            // Velocity mode (Standard 0 to Max)
            val maxVelPhys = engine.getInletVelocity() * 1.8f
            canvas.drawText("0.0", left, top - 15f, scaleTextPaint)
            val maxLabel = String.format(Locale.US, "%.1f m/s", maxVelPhys)
            canvas.drawText(maxLabel, right, top - 15f, scaleTextPaint)
        } else {
            // Pressure mode (Relative deviation)
            // LBM Pressure = rho * cs^2. Deviation = delta_rho * cs^2.
            // Phys pressure = LBM_Pressure * rho_phys * (dx/dt)^2
            // Our viz uses a range based on pressScale = 1 / (uIn^2 * 3).
            // A deviation of 0.5 (full scale from center) maps to deltaP = 0.5 / pressScale.
            val dx = engine.getDX()
            val dt = 0.000005f 
            val rhoPhys = engine.getDensity()
            val uInPhys = engine.getInletVelocity()
            val uInL = uInPhys * dt / dx
            
            // pressScale = 1 / (3 * uInL^2)
            // fullScaleDeltaP (Lattice) = 0.5 / pressScale = 1.5 * uInL^2
            // fullScaleDeltaP (Phys) = (1.5 * uInL^2) * rhoPhys * (dx/dt)^2
            val maxDeltaP = 1.5f * (uInL * uInL) * rhoPhys * (dx / dt) * (dx / dt)
            
            val offset = if (useAbsolutePressure) 101325f else 0.0f
            
            canvas.drawText(String.format(Locale.US, "%.0f Pa", offset - maxDeltaP), left, top - 15f, scaleTextPaint)
            canvas.drawText(String.format(Locale.US, "%.0f Pa", offset + maxDeltaP), right, top - 15f, scaleTextPaint)
        }
        
        // Unit Label
        val label = when (visualizationMode) {
            NativeLBMEngine.VIZ_VELOCITY -> "Air Velocity"
            NativeLBMEngine.VIZ_PRESSURE -> if (useAbsolutePressure) "Static Pressure (Absolute)" else "Static Pressure (Gauge)"
            NativeLBMEngine.VIZ_TOTAL_PRESSURE -> if (useAbsolutePressure) "Total Pressure (Absolute)" else "Total Pressure"
            else -> "Fluid Map"
        }
        canvas.drawText(label, (left + right) / 2f, top - 15f, scaleTextPaint)
    }

    override fun surfaceCreated(holder: SurfaceHolder) {
        running = true
        thread = Thread(this).apply { start() }
    }

    override fun surfaceChanged(holder: SurfaceHolder, format: Int, w: Int, h: Int) {}

    override fun surfaceDestroyed(holder: SurfaceHolder) {
        running = false
        thread?.join()
    }
    fun reset() {
        engine.resetSimulation()
    }

    fun reinit(w: Int, h: Int) {
        pendingReinit = ReinitParams(w, h)
    }

    fun setAirflowSpeed(speed: Float) {
        currentPhysVelocity = speed
        engine.setInletVelocity(speed)
    }

    fun setDensity(density: Float) {
        currentPhysDensity = density
        engine.setDensity(density)
    }

    fun setViscosity(viscosity: Float) {
        currentPhysViscosity = viscosity
        engine.setViscosity(viscosity)
    }

    fun getAirflowSpeed(): Float = currentPhysVelocity

    fun getDensity(): Float {
        return engine.getDensity()
    }

    fun getViscosity(): Float {
        return engine.getViscosity()
    }

    fun updateVisualizationMode(mode: Int) {
        visualizationMode = mode
        engine.setVisualizationMode(mode)
    }
}