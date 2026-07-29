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

    data class ForcePoint(val time: Float, val drag: Float, val lift: Float)
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

        // Find scale ranges
        var minF = 0f
        var maxF = 10f // Baseline
        history.forEach {
            maxF = maxOf(maxF, it.drag, it.lift)
            minF = minOf(minF, it.drag, it.lift)
        }
        val rangeF = (maxF - minF).coerceAtLeast(1f)
        
        val minTime = history.first().time
        val maxTime = history.last().time
        val rangeTime = (maxTime - minTime).coerceAtLeast(0.001f)

        // Draw Axes and Grid
        canvas.drawLine(padding, padding, padding, height - padding, textPaint) // Y Axis
        canvas.drawLine(padding, height - padding, width - padding, height - padding, textPaint) // X Axis

        // Draw horizontal grid lines and labels
        val gridLines = 5
        for (i in 0..gridLines) {
            val y = height - padding - (i.toFloat() / gridLines) * graphHeight
            val value = minF + (i.toFloat() / gridLines) * rangeF
            canvas.drawLine(padding, y, width - padding, y, gridPaint)
            canvas.drawText(String.format(Locale.US, "%.0f N", value), 10f, y + 10f, textPaint)
        }

        // Draw vertical grid lines and time labels
        val timeLines = 4
        for (i in 0..timeLines) {
            val x = padding + (i.toFloat() / timeLines) * graphWidth
            val timeValue = minTime + (i.toFloat() / timeLines) * rangeTime
            canvas.drawLine(x, padding, x, height - padding, gridPaint)
            canvas.drawText(String.format(Locale.US, "%.3fs", timeValue), x - 40f, height - padding + 40f, textPaint)
        }

        // Prepare Paths
        dragPath.reset()
        liftPath.reset()

        history.forEachIndexed { i, pt ->
            val x = padding + ((pt.time - minTime) / rangeTime) * graphWidth
            val yDrag = height - padding - ((pt.drag - minF) / rangeF) * graphHeight
            val yLift = height - padding - ((pt.lift - minF) / rangeF) * graphHeight

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

        // Legend - With clearer labels and indicators
        val legendY = padding - 40f
        
        // Drag Indicator
        val dragText = "Drag (Red)"
        canvas.drawText(dragText, padding, legendY, textPaint)
        canvas.drawLine(padding, legendY + 10f, padding + 150f, legendY + 10f, dragPaint)
        
        // Lift Indicator
        val liftText = "Lift (Cyan)"
        canvas.drawText(liftText, padding + 250f, legendY, textPaint)
        canvas.drawLine(padding + 250f, legendY + 10f, padding + 400f, legendY + 10f, liftPaint)
        
        canvas.drawText(String.format(Locale.US, "Total Window: %.2fs", rangeTime), width - padding - 300f, legendY, textPaint)
    }
}