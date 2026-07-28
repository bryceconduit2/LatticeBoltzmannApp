package com.example.latticeboltzmann

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.SurfaceHolder
import android.view.SurfaceView

class WindTunnelView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null
) : SurfaceView(context, attrs), SurfaceHolder.Callback, Runnable {
    // High resolution supported natively by C++ OpenMP
    private val simWidth = 800
    private val simHeight = 400
    private val engine = NativeLBMEngine(simWidth, simHeight)

    private var thread: Thread? = null
    @Volatile private var running = false

    // --- NEW: Variables to track touch state across threads ---
    @Volatile private var isHolding = false
    @Volatile private var touchSimX = 0
    @Volatile private var touchSimY = 0
    private var lastTouchSimX = -1
    private var lastTouchSimY = -1
    private var currentRadius = 8
    private val baseRadius = 8
    private val maxRadius = 60 // Stop growing so it doesn't block the whole tunnel

    private val bitmap = Bitmap.createBitmap(simWidth, simHeight, Bitmap.Config.ARGB_8888)
    private val paint = Paint().apply { isFilterBitmap = false } // Keep pixels sharp

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

    init {
        holder.addCallback(this)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val scaleX = width.toFloat() / simWidth
        val scaleY = height.toFloat() / simHeight

        when (event.action) {
            MotionEvent.ACTION_DOWN -> {
                isHolding = true
                currentRadius = baseRadius // Reset to starting size on a new tap
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

    override fun run() {
        while (running) {
            if (isHolding) {
                val dx = (touchSimX - lastTouchSimX).toFloat()
                val dy = (touchSimY - lastTouchSimY).toFloat()
                val dist = Math.sqrt((dx * dx + dy * dy).toDouble())

                if (dist > 2.0) {
                    // Moved significantly: Act as a brush
                    currentRadius = baseRadius
                    
                    // Linear interpolation to fill gaps
                    val steps = Math.ceil(dist / (baseRadius / 2.0)).toInt().coerceAtLeast(1)
                    for (i in 1..steps) {
                        val t = i.toFloat() / steps
                        val px = (lastTouchSimX + dx * t).toInt()
                        val py = (lastTouchSimY + dy * t).toInt()
                        engine.addObstacle(px, py, currentRadius)
                    }
                } else {
                    // Stationary: Grow the current point faster
                    if (currentRadius < maxRadius) {
                        currentRadius += 2 // Increased growth speed
                        if (currentRadius > maxRadius) currentRadius = maxRadius
                    }
                    engine.addObstacle(touchSimX, touchSimY, currentRadius)
                }
                
                lastTouchSimX = touchSimX
                lastTouchSimY = touchSimY
            }

            // Execute 15 physics steps natively before rendering
            engine.stepAndRender(bitmap, 15)

            val canvas = holder.lockCanvas()
            if (canvas != null) {
                canvas.drawBitmap(bitmap, null, Rect(0, 0, width, height), paint)
                
                // Draw a visual preview of the growing circle
                if (isHolding) {
                    val scaleX = width.toFloat() / simWidth
                    val scaleY = height.toFloat() / simHeight
                    canvas.drawCircle(
                        touchSimX * scaleX, 
                        touchSimY * scaleY, 
                        currentRadius * scaleX, 
                        previewPaint
                    )
                }

                canvas.drawText("(800x400)", 30f, 80f, textPaint)
                canvas.drawText("15 steps/frame", 30f, 140f, textPaint)
                holder.unlockCanvasAndPost(canvas)
            }
        }
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

    fun setAirflowSpeed(speed: Float) {
        engine.setInletVelocity(speed)
    }
}