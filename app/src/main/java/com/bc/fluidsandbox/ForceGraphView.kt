package com.bc.fluidsandbox

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View
import java.util.Locale

/**
 * Custom View that renders high-precision physics telemetry as a real-time plot.
 * 
 * It supports plotting both aerodynamic forces (Newtons) and coefficients (Cd, Cl).
 * It automatically scales the Y-axis based on the historical data range.
 */
class ForceGraphView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    // Toggle between raw Newton forces and normalized coefficients
    enum class DisplayMode { FORCE, COEFFICIENT }
    var displayMode = DisplayMode.FORCE
    
    // Data structure for a single simulation point
    data class ForcePoint(val time: Float, val drag: Float, val lift: Float, val cd: Float, val cl: Float, val isValid: Boolean = true)
    private var history = listOf<ForcePoint>()

    // Utility for density-independent pixel scaling
    private fun dpToPx(dp: Float): Float = dp * resources.displayMetrics.density
    private fun spToPx(sp: Float): Float = sp * resources.displayMetrics.scaledDensity

    // --- Paint Definitions ---
    private val dragPaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.STROKE
        strokeWidth = dpToPx(2f)
        isAntiAlias = true
    }

    private val liftPaint = Paint().apply {
        color = Color.CYAN
        style = Paint.Style.STROKE
        strokeWidth = dpToPx(2f)
        isAntiAlias = true
    }

    private val gridPaint = Paint().apply {
        color = Color.GRAY
        alpha = 100
        strokeWidth = dpToPx(1f)
    }

    private val textPaint = Paint().apply {
        color = Color.WHITE
        textSize = spToPx(12f)
        isAntiAlias = true
    }

    private val zeroLinePaint = Paint().apply {
        color = Color.WHITE
        alpha = 150
        strokeWidth = dpToPx(1.5f)
        pathEffect = DashPathEffect(floatArrayOf(dpToPx(4f), dpToPx(4f)), 0f)
    }

    private val dragPath = Path()
    private val liftPath = Path()

    /**
     * Updates the local data list and triggers a re-draw.
     */
    fun updateData(newHistory: List<ForcePoint>) {
        history = newHistory.toList()
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (history.isEmpty()) return

        // Define graph boundaries
        val padding = dpToPx(48f)
        val graphWidth = width - 2 * padding
        val graphHeight = height - 2 * padding

        // --- SCALE CALCULATION ---
        // Find the absolute min/max values in the current history to auto-scale the graph
        var minVal = 0f
        var maxVal = if (displayMode == DisplayMode.FORCE) 10f else 2f
        
        history.forEach {
            if (it.isValid) {
                if (displayMode == DisplayMode.FORCE) {
                    maxVal = maxOf(maxVal, it.drag, it.lift)
                    minVal = minOf(minVal, it.drag, it.lift)
                } else {
                    maxVal = maxOf(maxVal, it.cd, it.cl)
                    minVal = minOf(minVal, it.cd, it.cl)
                }
            }
        }
        val rangeVal = (maxVal - minVal).coerceAtLeast(0.001f)
        
        val minTime = history.first().time
        val maxTime = history.last().time
        val rangeTime = (maxTime - minTime).coerceAtLeast(0.001f)

        // --- GRID & AXES ---
        canvas.drawLine(padding, padding, padding, height - padding, textPaint) // Y Axis
        canvas.drawLine(padding, height - padding, width - padding, height - padding, textPaint) // X Axis

        // Draw horizontal grid lines and data labels
        val gridLines = 5
        val unitLabel = if (displayMode == DisplayMode.FORCE) "N" else ""
        
        for (i in 0..gridLines) {
            val y = height - padding - (i.toFloat() / gridLines) * graphHeight
            val value = minVal + (i.toFloat() / gridLines) * rangeVal
            canvas.drawLine(padding, y, width - padding, y, gridPaint)
            
            val labelText = if (displayMode == DisplayMode.FORCE) {
                String.format(Locale.US, "%.0f %s", value, unitLabel)
            } else {
                String.format(Locale.US, "%.2f", value)
            }
            canvas.drawText(labelText, dpToPx(4f), y + dpToPx(4f), textPaint)
        }

        // Draw vertical grid lines and time signatures
        val timeLines = 4
        for (i in 0..timeLines) {
            val x = padding + (i.toFloat() / timeLines) * graphWidth
            val timeValue = minTime + (i.toFloat() / timeLines) * rangeTime
            canvas.drawLine(x, padding, x, height - padding, gridPaint)
            canvas.drawText(String.format(Locale.US, "%.3fs", timeValue), x - dpToPx(16f), height - padding + dpToPx(20f), textPaint)
        }

        // Draw Zero Reference Line (dashed)
        if (minVal <= 0f && maxVal >= 0f) {
            val yZero = height - padding - ((0f - minVal) / rangeVal) * graphHeight
            canvas.drawLine(padding, yZero, width - padding, yZero, zeroLinePaint)
        }

        // --- PATH GENERATION ---
        dragPath.reset()
        liftPath.reset()

        var dragStarted = false
        var liftStarted = false

        history.forEach { pt ->
            if (pt.isValid) {
                // Map data coordinates to pixel coordinates
                val x = padding + ((pt.time - minTime) / rangeTime) * graphWidth
                val vDrag = if (displayMode == DisplayMode.FORCE) pt.drag else pt.cd
                val vLift = if (displayMode == DisplayMode.FORCE) pt.lift else pt.cl

                val yDrag = height - padding - ((vDrag - minVal) / rangeVal) * graphHeight
                val yLift = height - padding - ((vLift - minVal) / rangeVal) * graphHeight

                if (!dragStarted) {
                    dragPath.moveTo(x, yDrag)
                    dragStarted = true
                } else {
                    dragPath.lineTo(x, yDrag)
                }

                if (!liftStarted) {
                    liftPath.moveTo(x, yLift)
                    liftStarted = true
                } else {
                    liftPath.lineTo(x, yLift)
                }
            } else {
                dragStarted = false
                liftStarted = false
            }
        }

        // Commit paths to canvas
        canvas.drawPath(dragPath, dragPaint)
        canvas.drawPath(liftPath, liftPaint)

        // --- STATISTICS CALCULATION ---
        // Calculate averages, skipping the initial turbulence to allow settling
        var sumDrag = 0f
        var sumLift = 0f
        var sumCd = 0f
        var sumCl = 0f
        var count = 0
        
        history.forEach {
            if (it.time > 0.05f && it.isValid) {
                sumDrag += it.drag
                sumLift += it.lift
                sumCd += it.cd
                sumCl += it.cl
                count++
            }
        }
        
        val avgDrag = if (count > 0) sumDrag / count else 0f
        val avgLift = if (count > 0) sumLift / count else 0f
        val avgCd = if (count > 0) sumCd / count else 0f
        val avgCl = if (count > 0) sumCl / count else 0f

        // --- LEGEND RENDERING ---
        val legendY = padding - dpToPx(16f)
        val modeLabel = if (displayMode == DisplayMode.FORCE) "Forces" else "Coefficients"
        
        val displayDrag: String
        val displayLift: String
        
        if (count == 0) {
            displayDrag = "Settling..."
            displayLift = "Settling..."
        } else {
            val lastPt = history.last()
            if (!lastPt.isValid) {
                displayDrag = "N/A (Touching Boundary)"
                displayLift = "N/A (Touching Boundary)"
            } else if (displayMode == DisplayMode.FORCE) {
                displayDrag = String.format(Locale.US, "Avg Drag: %.1f N", avgDrag)
                displayLift = String.format(Locale.US, "Avg Lift: %.1f N", avgLift)
            } else {
                displayDrag = String.format(Locale.US, "Avg Cd: %.2f", avgCd)
                displayLift = String.format(Locale.US, "Avg Cl: %.2f", avgCl)
            }
        }
        
        val dragIndicatorWidth = dpToPx(60f)
        val liftOffset = dpToPx(140f)
        val infoOffset = dpToPx(280f)

        canvas.drawText(displayDrag, padding, legendY, textPaint)
        canvas.drawLine(padding, legendY + dpToPx(4f), padding + dragIndicatorWidth, legendY + dpToPx(4f), dragPaint)
        
        canvas.drawText(displayLift, padding + liftOffset, legendY, textPaint)
        canvas.drawLine(padding + liftOffset, legendY + dpToPx(4f), padding + liftOffset + dragIndicatorWidth, legendY + dpToPx(4f), liftPaint)
        
        canvas.drawText(String.format(Locale.US, "Mode: %s | Window: %.2fs", modeLabel, rangeTime), padding + infoOffset, legendY, textPaint)
    }
}
