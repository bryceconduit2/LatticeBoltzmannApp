package com.BC.latticeboltzmann

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
    enum class BrushShape { CIRCLE, SQUARE, NACA0012, NACA4412 }
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
    private var persistentObjectRadius = 8
    private var hasActiveObject = false
    private val maxRadius = 60 // Stop growing so it doesn't block the whole tunnel
    
    var brushShape = BrushShape.CIRCLE
    var showDetailedTelemetry = true
    var showGridlines = false
    var baseBrushSize = 8
    var airfoilAoA = 0.0f
    var visualizationMode = NativeLBMEngine.VIZ_VELOCITY
    var boundaryMode = NativeLBMEngine.BND_PERIODIC
    var useAbsolutePressure = false
    var coreCount = 4 // Default to a safe 4 cores

    // Telemetry
    private var lastTime = System.nanoTime()
    private var fps = 0.0
    private var sps = 0.0
    private var stepsPerFrame = 12

    private var bitmap = Bitmap.createBitmap(simWidth, simHeight, Bitmap.Config.ARGB_8888)
    private val paint = Paint().apply { isFilterBitmap = false } // Keep pixels sharp
    
    // Parameter Persistence
    private var currentPhysVelocity = 30.0f
    private var currentPhysDensity = 1.225f
    private var currentPhysViscosity = 1.5e-5f
    
    private val forceHistory = mutableListOf<ForceGraphView.ForcePoint>()
    private val maxHistorySize = 500

    private fun dpToPx(dp: Float): Float = dp * resources.displayMetrics.density
    private fun spToPx(sp: Float): Float = sp * resources.displayMetrics.scaledDensity

    private val textPaint = Paint().apply {
        color = Color.WHITE
        textSize = spToPx(18f)
        isAntiAlias = true
        setShadowLayer(3f, 1f, 1f, Color.BLACK)
    }

    private val gridPaintOverlay = Paint().apply {
        color = Color.BLACK
        alpha = 80 // Increased slightly for better contrast
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
                engine.setBoundaryMode(boundaryMode)
                engine.setNumThreads(coreCount)
                
                pendingReinit = null
            }

            if (isHolding) {
                val dx = (touchSimX - lastTouchSimX).toFloat()
                val dy = (touchSimY - lastTouchSimY).toFloat()
                val dist = hypot(dx, dy)

                if (dist > 0.5f) {
                    // Moved significantly: Act as a brush. 
                    // Reset to base size immediately for uniform lines.
                    currentRadius = baseBrushSize
                    
                    // High-density interpolation to ensure no gaps even at high speed
                    val stepSize = (baseBrushSize / 4.0).coerceAtLeast(1.0)
                    val steps = Math.ceil(dist / stepSize).toInt().coerceAtLeast(1)
                    
                    for (i in 0..steps) {
                        val t = i.toFloat() / steps
                        val px = (lastTouchSimX + dx * t).toInt()
                        val py = (lastTouchSimY + dy * t).toInt()
                        addObstacleToEngine(px, py, currentRadius)
                    }
                } else {
                    // Stationary: Grow the current point for large obstacles
                    if (currentRadius < maxRadius) {
                        currentRadius += 1 // Slower growth for better precision
                        if (currentRadius > maxRadius) currentRadius = maxRadius
                    }
                    addObstacleToEngine(touchSimX, touchSimY, currentRadius)
                }
                
                persistentObjectRadius = currentRadius
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
            
            // Collect data for the graph (using instant values to see vortex oscillations)
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

            val canvas = holder.lockCanvas()
            if (canvas != null) {
                canvas.drawBitmap(bitmap, null, Rect(0, 0, width, height), paint)
                
                if (showGridlines) {
                    drawGrid(canvas)
                }
                
                // Draw a visual preview of the growing shape
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
                            // Draw a simple line to represent the airfoil chord for preview
                            val angleRad = Math.toRadians(airfoilAoA.toDouble())
                            val endX = cx + (currentRadius * scaleX * 5.0f * Math.cos(angleRad)).toFloat()
                            val endY = cy + (currentRadius * scaleX * 5.0f * Math.sin(angleRad)).toFloat()
                            canvas.drawLine(cx, cy, endX, endY, previewPaint)
                        }
                    }
                }

                val currentVel = engine.getInletVelocity()
                val dragN = engine.getDragForce()
                val dragCd = engine.getDragCoefficient()
                val liftN = engine.getLiftForce()
                val liftCl = engine.getLiftCoefficient()
                val elapsedSeconds = engine.getTotalSteps() * 0.000005f
                
                val telemetryPadding = dpToPx(24f)
                val telemetrySpacing = dpToPx(28f)
                
                canvas.drawText(String.format(Locale.US, "Velocity: %.1f m/s | Time: %.2fs", currentVel, elapsedSeconds), telemetryPadding, telemetryPadding * 1.5f, textPaint)
                canvas.drawText(String.format(Locale.US, "Drag: %.1f N | Cd: %.2f", dragN, dragCd), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing, textPaint)
                canvas.drawText(String.format(Locale.US, "Lift: %.1f N | Cl: %.2f", liftN, liftCl), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 2f, textPaint)
                
                // Reynolds Number Calculation (Holistic)
                val dx = engine.getDX()
                val viscosity = engine.getViscosity()
                
                // Use the actual horizontal span (chord) of the entire object set
                val L_obj = engine.getHorizontalSpan()
                val reObject = if (viscosity > 0 && hasActiveObject) (currentVel * L_obj) / viscosity else 0f
                
                val reValueStr = if (hasActiveObject) String.format(Locale.US, "%,.0f", reObject) else "--"
                canvas.drawText(String.format(Locale.US, "Reynolds No (Object): %s", reValueStr), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 3f, textPaint)

                if (showDetailedTelemetry) {
                    val physW = simWidth * dx
                    val physH = simHeight * dx
                    val density = engine.getDensity()
                    val activeCores = getActiveCoreCount()

                    canvas.drawText(String.format(Locale.US, "Tunnel: %.1fm x %.1fm (1.0m Depth)", physW, physH), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 5.5f, textPaint)
                    canvas.drawText(String.format(Locale.US, "Density: %.2f kg/m³ | Visc: %.1e m²/s", density, viscosity), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 6.5f, textPaint)
                    canvas.drawText(String.format(Locale.US, "Performance: %.0f FPS | %.0f Steps/s", fps, sps), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 7.5f, textPaint)
                    canvas.drawText(String.format(Locale.US, "Active Cores: %d / %d", activeCores, coreCount), telemetryPadding, telemetryPadding * 1.5f + telemetrySpacing * 8.5f, textPaint)
                }

                drawScaleBar(canvas)
                
                holder.unlockCanvasAndPost(canvas)
            }
        }
    }

    private fun addObstacleToEngine(x: Int, y: Int, radius: Int) {
        hasActiveObject = true
        when (brushShape) {
            BrushShape.CIRCLE -> engine.addObstacle(x, y, radius)
            BrushShape.SQUARE -> engine.addBoxObstacle(x, y, radius * 2)
            BrushShape.NACA0012 -> engine.addNacaAirfoil(x, y, radius * 5, 0.0f, 0.0f, 0.12f, airfoilAoA)
            BrushShape.NACA4412 -> engine.addNacaAirfoil(x, y, radius * 5, 0.04f, 0.4f, 0.12f, airfoilAoA)
        }
    }

    private fun drawScaleBar(canvas: Canvas) {
        val barWidth = dpToPx(240f)
        val barHeight = dpToPx(16f)
        val margin = dpToPx(20f)
        
        val left = width - barWidth - margin
        val right = width - margin
        val bottom = height - margin - dpToPx(8f) 
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
        previewPaint.strokeWidth = dpToPx(1f)
        canvas.drawRect(left, top, right, bottom, previewPaint)

        // Draw Labels
        val labelMargin = dpToPx(6f)
        if (visualizationMode == NativeLBMEngine.VIZ_VELOCITY) {
            // Velocity mode (Standard 0 to Max)
            val maxVelPhys = engine.getInletVelocity() * 1.8f
            canvas.drawText("0.0", left, top - labelMargin, scaleTextPaint)
            val maxLabel = String.format(Locale.US, "%.1f m/s", maxVelPhys)
            canvas.drawText(maxLabel, right, top - labelMargin, scaleTextPaint)
        } else {
            // Pressure mode (Relative deviation)
            val rhoPhys = engine.getDensity()
            val uInPhys = engine.getInletVelocity()
            val maxDeltaP = 0.5f * rhoPhys * uInPhys * uInPhys
            
            val offset = if (useAbsolutePressure) 101325f else 0.0f
            
            canvas.drawText(String.format(Locale.US, "%.0f Pa", offset - maxDeltaP), left, top - labelMargin, scaleTextPaint)
            canvas.drawText(String.format(Locale.US, "%.0f Pa", offset + maxDeltaP), right, top - labelMargin, scaleTextPaint)
        }
        
        // Unit Label
        val label = when (visualizationMode) {
            NativeLBMEngine.VIZ_VELOCITY -> "Air Velocity"
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

    fun getActiveCoreCount(): Int = engine.getActiveCores()

    fun getMaxAvailableCores(): Int = engine.getMaxCores()

    fun getSimWidth(): Int = simWidth

    fun getForceHistory(): List<ForceGraphView.ForcePoint> {
        return synchronized(forceHistory) {
            forceHistory.toList()
        }
    }

    private fun drawGrid(canvas: Canvas) {
        val scaleX = width.toFloat() / simWidth
        val scaleY = height.toFloat() / simHeight
        
        // dx = 0.0025m (2.5mm)
        // 40 cells = 0.1m (10cm)
        val gridIntervalCells = 40
        val gridStepX = gridIntervalCells * scaleX
        val gridStepY = gridIntervalCells * scaleY
        
        // Vertical lines
        var x = 0f
        while (x < width) {
            canvas.drawLine(x, 0f, x, height.toFloat(), gridPaintOverlay)
            x += gridStepX
        }
        
        // Horizontal lines
        var y = 0f
        while (y < height) {
            canvas.drawLine(0f, y, width.toFloat(), y, gridPaintOverlay)
            y += gridStepY
        }
    }
}