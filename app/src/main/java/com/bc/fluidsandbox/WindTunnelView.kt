package com.bc.fluidsandbox

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.SurfaceHolder
import android.view.SurfaceView
import java.util.Locale
import kotlin.math.hypot

/**
 * The core Interactive UI component of the Fluid Sandbox.
 * 
 * This class manages a persistent background thread that runs the physics engine, 
 * handles complex touch interactions for drawing shapes, and renders the fluid
 * velocity map directly to the screen via a SurfaceView.
 */
class WindTunnelView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : SurfaceView(context, attrs), SurfaceHolder.Callback, Runnable {
    
    // Shape definitions for drawing
    enum class BrushShape { CIRCLE, SQUARE, NACA }
    private data class ReinitParams(val w: Int, val h: Int)
    
    // Grid dimensions (Default 400x200 = 80,000 fluid cells)
    private var simWidth = 400
    private var simHeight = 200
    private var engine = NativeLBMEngine(simWidth, simHeight)
    @Volatile private var pendingReinit: ReinitParams? = null

    // Multi-threading state
    private var thread: Thread? = null
    @Volatile private var running = false

    // --- Touch Interaction State ---
    @Volatile private var isHolding = false
    @Volatile private var touchSimX = 0
    @Volatile private var touchSimY = 0
    private var lastTouchSimX = -1
    private var lastTouchSimY = -1
    private var currentRadius = 8
    private var persistentObjectRadius = 8
    private var hasActiveObject = false
    private val maxRadius = 60 // Max growth limit to prevent blocking the entire tunnel
    
    // --- Current Physics Settings ---
    var brushShape = BrushShape.CIRCLE
    var nacaM = 0.0f
    var nacaP = 0.0f
    var nacaT = 0.12f
    
    var showDetailedTelemetry = true
    var showGridlines = false
    var baseBrushSize = 8
    var airfoilAoA = 0.0f
    var visualizationMode = NativeLBMEngine.VIZ_VELOCITY
    var boundaryMode = NativeLBMEngine.BND_PERIODIC
    var useAbsolutePressure = false
    var coreCount = 4
    var simulationDtScale = 1.0f

    // Telemetry accumulators
    private var lastTime = System.nanoTime()
    private var fps = 0.0
    private var sps = 0.0
    private var stepsPerFrame = 12

    // Rendering assets
    private var bitmap = Bitmap.createBitmap(simWidth, simHeight, Bitmap.Config.ARGB_8888)
    private val paint = Paint().apply { isFilterBitmap = false } // Keep simulation pixels crisp
    
    // Physical state persistence
    private var currentPhysVelocity = 30.0f
    private var currentPhysDensity = 1.225f
    private var currentPhysViscosity = 1.5e-5f
    
    // Data stream for the real-time graph
    private val forceHistory = mutableListOf<ForceGraphView.ForcePoint>()
    private val maxHistorySize = 500

    // Scaling helpers
    private fun dpToPx(dp: Float): Float = dp * resources.displayMetrics.density
    private fun spToPx(sp: Float): Float = sp * resources.displayMetrics.scaledDensity

    // --- HUD & Overlay Paints ---
    private val textPaint = Paint().apply {
        color = Color.WHITE
        textSize = spToPx(14f)
        isAntiAlias = true
        setShadowLayer(3f, 1f, 1f, Color.BLACK)
    }

    private val gridPaintOverlay = Paint().apply {
        color = Color.BLACK
        alpha = 80
        strokeWidth = dpToPx(1f)
        style = Paint.Style.STROKE
    }

    private val previewPaint = Paint().apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = dpToPx(2f)
        isAntiAlias = true
    }

    private val scalePaint = Paint().apply {
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    private val scaleTextPaint = Paint().apply {
        color = Color.WHITE
        textSize = spToPx(12f)
        isAntiAlias = true
        textAlign = Paint.Align.CENTER
        setShadowLayer(2f, 1f, 1f, Color.BLACK)
    }

    // Gradient colors for the velocity scale bar
    private val scaleColors = intArrayOf(
        Color.BLUE, Color.CYAN, Color.GREEN, Color.YELLOW, Color.RED
    )
    private val scalePositions = floatArrayOf(
        0.0f, 0.25f, 0.5f, 0.75f, 1.0f
    )

    init {
        holder.addCallback(this)
    }

    /**
     * Handles complex multi-touch: 
     * - Single tap adds an object.
     * - Long press grows the object size.
     * - Dragging acts as a brush.
     */
    override fun onTouchEvent(event: MotionEvent): Boolean {
        val scaleX = width.toFloat() / simWidth
        val scaleY = height.toFloat() / simHeight

        when (event.action) {
            MotionEvent.ACTION_DOWN -> {
                performClick()
                isHolding = true
                currentRadius = baseBrushSize 
                touchSimX = (event.x / scaleX).toInt()
                touchSimY = (event.y / scaleY).toInt()
                lastTouchSimX = touchSimX
                lastTouchSimY = touchSimY
                return true
            }
            MotionEvent.ACTION_MOVE -> {
                touchSimX = (event.x / scaleX).toInt()
                touchSimY = (event.y / scaleY).toInt()
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                isHolding = false 
                return true
            }
        }
        return super.onTouchEvent(event)
    }

    override fun performClick(): Boolean {
        super.performClick()
        return true
    }

    /**
     * MAIN PHYSICS THREAD LOOP.
     * Runs continuously at the maximum possible speed allowed by the CPU.
     */
    override fun run() {
        while (running) {
            // --- THREAD SAFE RE-INITIALIZATION ---
            pendingReinit?.let { params ->
                simWidth = params.w
                simHeight = params.h
                bitmap = Bitmap.createBitmap(simWidth, simHeight, Bitmap.Config.ARGB_8888)
                engine = NativeLBMEngine(simWidth, simHeight)
                
                // Re-apply settings to the new engine instance
                // DeltaTime must be set BEFORE Velocity for correct conversion
                engine.setDeltaTime(0.000005f * simulationDtScale)
                engine.setInletVelocity(currentPhysVelocity)
                engine.setDensity(currentPhysDensity)
                engine.setViscosity(currentPhysViscosity)
                engine.setVisualizationMode(visualizationMode)
                engine.setBoundaryMode(boundaryMode)
                engine.setNumThreads(coreCount)
                
                pendingReinit = null
            }

            // --- DRAWING LOGIC ---
            if (isHolding) {
                val dx = (touchSimX - lastTouchSimX).toFloat()
                val dy = (touchSimY - lastTouchSimY).toFloat()
                val dist = hypot(dx, dy)

                if (dist > 0.5f) {
                    // INTERPOLATED BRUSH: Ensures no gaps if the user moves their finger quickly
                    currentRadius = baseBrushSize
                    val stepSize = (baseBrushSize / 4.0).coerceAtLeast(1.0)
                    val steps = Math.ceil(dist / stepSize).toInt().coerceAtLeast(1)
                    
                    for (i in 0..steps) {
                        val t = i.toFloat() / steps
                        val px = (lastTouchSimX + dx * t).toInt()
                        val py = (lastTouchSimY + dy * t).toInt()
                        addObstacleToEngine(px, py, currentRadius)
                    }
                } else {
                    // STATIONARY GROWTH: Slowly grows the object if the user holds their finger still
                    if (currentRadius < maxRadius) {
                        currentRadius += 1 
                        if (currentRadius > maxRadius) currentRadius = maxRadius
                    }
                    addObstacleToEngine(touchSimX, touchSimY, currentRadius)
                }
                
                persistentObjectRadius = currentRadius
                lastTouchSimX = touchSimX
                lastTouchSimY = touchSimY
            }

            // --- ENGINE EXECUTION ---
            val now = System.nanoTime()
            val dtNano = now - lastTime
            if (dtNano > 0) {
                val currentFps = 1_000_000_000.0 / dtNano
                fps = fps * 0.9 + currentFps * 0.1 // EMA Filtered FPS
                sps = fps * stepsPerFrame
            }
            lastTime = now

            // TRIGGER THE C++ PHYSICS KERNEL
            engine.stepAndRender(bitmap, stepsPerFrame)
            
            // STREAM TELEMETRY TO GRAPH
            val elapsedSec = engine.getTotalSteps() * 0.000005f
            val dragN = engine.getDragForce()
            val liftN = engine.getLiftForce()
            val cd = engine.getInstantDragCoefficient()
            val cl = engine.getInstantLiftCoefficient()
            
            synchronized(forceHistory) {
                forceHistory.add(ForceGraphView.ForcePoint(elapsedSec, dragN, liftN, cd, cl))
                if (forceHistory.size > maxHistorySize) {
                    forceHistory.removeAt(0)
                }
            }

            // --- UI RENDERING ---
            val canvas = holder.lockCanvas()
            if (canvas != null) {
                // 1. Draw the actual physics fluid map
                canvas.drawBitmap(bitmap, null, Rect(0, 0, width, height), paint)
                
                if (showGridlines) {
                    drawGrid(canvas)
                }
                
                // 2. Draw the visual touch preview (Outline)
                if (isHolding) {
                    val scaleX = width.toFloat() / simWidth
                    val scaleY = height.toFloat() / simHeight
                    val r = currentRadius * scaleX
                    val cx = touchSimX * scaleX
                    val cy = touchSimY * scaleY

                    when (brushShape) {
                        BrushShape.CIRCLE -> canvas.drawCircle(cx, cy, r, previewPaint)
                        BrushShape.SQUARE -> canvas.drawRect(cx - r, cy - r, cx + r, cy + r, previewPaint)
                        else -> {
                            val angleRad = Math.toRadians(airfoilAoA.toDouble())
                            val endX = cx + (currentRadius * scaleX * 5.0f * Math.cos(angleRad)).toFloat()
                            val endY = cy + (currentRadius * scaleX * 5.0f * Math.sin(angleRad)).toFloat()
                            canvas.drawLine(cx, cy, endX, endY, previewPaint)
                        }
                    }
                }

                // 3. Draw HUD (Velocity, Drag, Lift, Reynolds)
                val currentVel = engine.getInletVelocity()
                val dragN_hud = engine.getDragForce()
                val dragCd = engine.getDragCoefficient()
                val liftN_hud = engine.getLiftForce()
                val liftCl = engine.getLiftCoefficient()
                val elapsedSeconds = engine.getTotalSteps() * 0.000005f
                
                val telemetryPadding = dpToPx(24f)
                val telemetrySpacing = dpToPx(22f)
                
                canvas.drawText(String.format(Locale.US, "Velocity: %.1f m/s | Time: %.2fs", currentVel, elapsedSeconds), telemetryPadding, telemetryPadding * 1.5f, textPaint)
                canvas.drawText(String.format(Locale.US, "Drag: %.1f N | Cd: %.2f", dragN_hud, dragCd), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing, textPaint)
                canvas.drawText(String.format(Locale.US, "Lift: %.1f N | Cl: %.2f", liftN_hud, liftCl), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 2f, textPaint)
                
                // ADVANCED REYNOLDS CALCULATION (Dynamic Span)
                val dx = engine.getDX()
                val viscosity = engine.getViscosity()
                val L_obj = engine.getHorizontalSpan()
                val reObject = if (viscosity > 0 && hasActiveObject) (currentVel * L_obj) / viscosity else 0f
                val reValueStr = if (hasActiveObject) String.format(Locale.US, "%,.0f", reObject) else "--"
                canvas.drawText(String.format(Locale.US, "~Reynolds No: %s", reValueStr), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 3f, textPaint)

                // 4. Draw Detailed Performance HUD
                if (showDetailedTelemetry) {
                    val physW = simWidth * dx
                    val physH = simHeight * dx
                    val activeCores = getActiveCoreCount()

                    canvas.drawText(String.format(Locale.US, "Tunnel: %.1fm x %.1fm (1.0m Depth)", physW, physH), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 4.5f, textPaint)
                    canvas.drawText(String.format(Locale.US, "Performance: %.0f FPS | %.0f Steps/s", fps, sps), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 5.5f, textPaint)
                    canvas.drawText(String.format(Locale.US, "Active Cores: %d", activeCores), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 6.5f, textPaint)
                }

                // 5. Draw Velocity Scale Bar
                drawScaleBar(canvas)
                
                holder.unlockCanvasAndPost(canvas)
            }
        }
    }

    /**
     * Translates UI coordinates to scientific grid obstacles.
     */
    private fun addObstacleToEngine(x: Int, y: Int, radius: Int) {
        hasActiveObject = true
        when (brushShape) {
            BrushShape.CIRCLE -> engine.addObstacle(x, y, radius)
            BrushShape.SQUARE -> engine.addBoxObstacle(x, y, radius * 2)
            BrushShape.NACA -> engine.addNacaAirfoil(x, y, radius * 5, nacaM, nacaP, nacaT, airfoilAoA)
        }
    }

    /**
     * Renders a professional color legend representing wind speed or pressure.
     */
    private fun drawScaleBar(canvas: Canvas) {
        val barWidth = dpToPx(240f)
        val barHeight = dpToPx(16f)
        val margin = dpToPx(28f) // Increased margin to prevent text cutoff
        
        val left = width - barWidth - margin
        val right = width - margin
        val bottom = height - margin - dpToPx(8f) 
        val top = bottom - barHeight

        if (scalePaint.shader == null) {
            scalePaint.shader = LinearGradient(
                left, 0f, right, 0f,
                scaleColors, scalePositions, Shader.TileMode.CLAMP
            )
        }

        canvas.drawRect(left, top, right, bottom, scalePaint)
        previewPaint.strokeWidth = dpToPx(1f)
        canvas.drawRect(left, top, right, bottom, previewPaint)

        val labelMargin = dpToPx(6f)
        if (visualizationMode == NativeLBMEngine.VIZ_VELOCITY) {
            val maxVelPhys = engine.getInletVelocity() * 1.8f
            
            // Left Label
            scaleTextPaint.textAlign = Paint.Align.LEFT
            canvas.drawText("0.0", left, top - labelMargin, scaleTextPaint)
            
            // Right Label
            scaleTextPaint.textAlign = Paint.Align.RIGHT
            val maxLabel = String.format(Locale.US, "%.1f m/s", maxVelPhys)
            canvas.drawText(maxLabel, right, top - labelMargin, scaleTextPaint)
        } else {
            val rhoPhys = engine.getDensity()
            val uInPhys = engine.getInletVelocity()
            val maxDeltaP = 0.5f * rhoPhys * uInPhys * uInPhys
            val offset = if (useAbsolutePressure) 101325f else 0.0f
            
            scaleTextPaint.textAlign = Paint.Align.LEFT
            canvas.drawText(String.format(Locale.US, "%.0f Pa", offset - maxDeltaP), left, top - labelMargin, scaleTextPaint)
            
            scaleTextPaint.textAlign = Paint.Align.RIGHT
            canvas.drawText(String.format(Locale.US, "%.0f Pa", offset + maxDeltaP), right, top - labelMargin, scaleTextPaint)
        }
        
        // Center Title
        scaleTextPaint.textAlign = Paint.Align.CENTER
        val label = when (visualizationMode) {
            NativeLBMEngine.VIZ_VELOCITY -> "Air Velocity (m/s)"
            NativeLBMEngine.VIZ_PRESSURE -> if (useAbsolutePressure) "Static Pressure (Absolute)" else "Static Pressure (Gauge)"
            NativeLBMEngine.VIZ_TOTAL_PRESSURE -> if (useAbsolutePressure) "Total Pressure (Absolute)" else "Total Pressure"
            else -> "Fluid Map"
        }
        canvas.drawText(label, (left + right) / 2f, top - labelMargin, scaleTextPaint)
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

    // --- External Controller Methods (Used by MainActivity) ---

    fun reset() {
        engine.resetSimulation()
        hasActiveObject = false
        synchronized(forceHistory) {
            forceHistory.clear()
        }
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

    fun updateBoundaryMode(mode: Int) {
        boundaryMode = mode
        engine.setBoundaryMode(mode)
    }

    fun updateCoreCount(count: Int) {
        coreCount = count
        engine.setNumThreads(count)
    }

    fun updateDtScale(scale: Float) {
        simulationDtScale = scale
        val baseDt = 0.000005f
        engine.setDeltaTime(baseDt * scale)
        // Re-apply the physical velocity setting so the engine updates its 
        // internal lattice speed calculation using the new time step.
        engine.setInletVelocity(currentPhysVelocity)
    }

    fun getActiveCoreCount(): Int = engine.getActiveCores()

    fun getMaxAvailableCores(): Int = engine.getMaxCores()

    fun getSimWidth(): Int = simWidth

    fun getForceHistory(): List<ForceGraphView.ForcePoint> {
        return synchronized(forceHistory) {
            forceHistory.toList()
        }
    }

    /**
     * Renders a 10cm grid overlay based on the dx constant (0.0025m).
     */
    private fun drawGrid(canvas: Canvas) {
        val scaleX = width.toFloat() / simWidth
        val scaleY = height.toFloat() / simHeight
        val gridIntervalCells = 40
        val gridStepX = gridIntervalCells * scaleX
        val gridStepY = gridIntervalCells * scaleY
        
        var x = 0f
        while (x < width) {
            canvas.drawLine(x, 0f, x, height.toFloat(), gridPaintOverlay)
            x += gridStepX
        }
        var y = 0f
        while (y < height) {
            canvas.drawLine(0f, y, width.toFloat(), y, gridPaintOverlay)
            y += gridStepY
        }
    }
}
