package com.example.latticeboltzmann

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View
import java.util.Locale

class ForceGraphView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    enum class DisplayMode { FORCE, COEFFICIENT }
    var displayMode = DisplayMode.FORCE
    
    data class ForcePoint(val time: Float, val drag: Float, val lift: Float, val cd: Float, val cl: Float)
    private var history = listOf<ForcePoint>()

    private val dragPaint = Paint().apply {
        color = Color.RED
        style = Paint.Style.STROKE
        strokeWidth = 5f
        isAntiAlias = true
    }

    private val liftPaint = Paint().apply {
        color = Color.CYAN
        style = Paint.Style.STROKE
        strokeWidth = 5f
        isAntiAlias = true
    }

    private val gridPaint = Paint().apply {
        color = Color.GRAY
        alpha = 100
        strokeWidth = 2f
    }

    private val textPaint = Paint().apply {
        color = Color.WHITE
        textSize = 30f
        isAntiAlias = true
    }

    private val zeroLinePaint = Paint().apply {
        color = Color.WHITE
        alpha = 150
        strokeWidth = 3f
        pathEffect = DashPathEffect(floatArrayOf(10f, 10f), 0f)
    }

    private val dragPath = Path()
    private val liftPath = Path()

    fun updateData(newHistory: List<ForcePoint>) {
        history = newHistory.toList()
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (history.isEmpty()) return

        val padding = 100f
        val graphWidth = width - 2 * padding
        val graphHeight = height - 2 * padding

        // Find scale ranges based on display mode
        var minVal = 0f
        var maxVal = if (displayMode == DisplayMode.FORCE) 10f else 2f
        
        history.forEach {
            if (displayMode == DisplayMode.FORCE) {
                maxVal = maxOf(maxVal, it.drag, it.lift)
                minVal = minOf(minVal, it.drag, it.lift)
            } else {
                maxVal = maxOf(maxVal, it.cd, it.cl)
                minVal = minOf(minVal, it.cd, it.cl)
            }
        }
        val rangeVal = (maxVal - minVal).coerceAtLeast(0.001f)
        
        val minTime = history.first().time
        val maxTime = history.last().time
        val rangeTime = (maxTime - minTime).coerceAtLeast(0.001f)

        // Draw Axes and Grid
        canvas.drawLine(padding, padding, padding, height - padding, textPaint) // Y Axis
        canvas.drawLine(padding, height - padding, width - padding, height - padding, textPaint) // X Axis

        // Draw horizontal grid lines and labels
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
            canvas.drawText(labelText, 10f, y + 10f, textPaint)
        }

        // Draw vertical grid lines and time labels
        val timeLines = 4
        for (i in 0..timeLines) {
            val x = padding + (i.toFloat() / timeLines) * graphWidth
            val timeValue = minTime + (i.toFloat() / timeLines) * rangeTime
            canvas.drawLine(x, padding, x, height - padding, gridPaint)
            canvas.drawText(String.format(Locale.US, "%.3fs", timeValue), x - 40f, height - padding + 40f, textPaint)
        }

        // Draw Zero Reference Line
        if (minVal <= 0f && maxVal >= 0f) {
            val yZero = height - padding - ((0f - minVal) / rangeVal) * graphHeight
            canvas.drawLine(padding, yZero, width - padding, yZero, zeroLinePaint)
        }

        // Prepare Paths
        dragPath.reset()
        liftPath.reset()

        history.forEachIndexed { i, pt ->
            val x = padding + ((pt.time - minTime) / rangeTime) * graphWidth
            
            val vDrag = if (displayMode == DisplayMode.FORCE) pt.drag else pt.cd
            val vLift = if (displayMode == DisplayMode.FORCE) pt.lift else pt.cl
            
            val yDrag = height - padding - ((vDrag - minVal) / rangeVal) * graphHeight
            val yLift = height - padding - ((vLift - minVal) / rangeVal) * graphHeight

            if (i == 0) {
                dragPath.moveTo(x, yDrag)
                liftPath.moveTo(x, yLift)
            } else {
                dragPath.lineTo(x, yDrag)
                liftPath.lineTo(x, yLift)
            }
        }

        canvas.drawPath(dragPath, dragPaint)
        canvas.drawPath(liftPath, liftPaint)

        // Calculate averages - skipping the first 0.05s of simulation time to allow settling
        var sumDrag = 0f
        var sumLift = 0f
        var sumCd = 0f
        var sumCl = 0f
        var count = 0
        
        history.forEach {
            if (it.time > 0.05f) {
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

        // Legend - With clearer labels and indicators
        val legendY = padding - 40f
        val modeLabel = if (displayMode == DisplayMode.FORCE) "Forces" else "Coefficients"
        
        val displayDrag: String
        val displayLift: String
        
        if (count == 0) {
            displayDrag = "Settling..."
            displayLift = "Settling..."
        } else {
            if (displayMode == DisplayMode.FORCE) {
                displayDrag = String.format(Locale.US, "Avg Drag: %.1f N", avgDrag)
                displayLift = String.format(Locale.US, "Avg Lift: %.1f N", avgLift)
            } else {
                displayDrag = String.format(Locale.US, "Avg Cd: %.2f", avgCd)
                displayLift = String.format(Locale.US, "Avg Cl: %.2f", avgCl)
            }
        }
        
        canvas.drawText(displayDrag, padding, legendY, textPaint)
        canvas.drawLine(padding, legendY + 10f, padding + 200f, legendY + 10f, dragPaint)
        
        canvas.drawText(displayLift, padding + 300f, legendY, textPaint)
        canvas.drawLine(padding + 300f, legendY + 10f, padding + 500f, legendY + 10f, liftPaint)
        
        canvas.drawText(String.format(Locale.US, "Mode: %s | Window: %.2fs", modeLabel, rangeTime), width - padding - 450f, legendY, textPaint)
    }
}