package com.example.latticeboltzmann

import android.os.Bundle
import android.view.LayoutInflater
import android.widget.Button
import android.widget.RadioGroup
import android.widget.SeekBar
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.google.android.material.bottomsheet.BottomSheetDialog
import java.util.Locale

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        supportActionBar?.hide()
        setContentView(R.layout.activity_main)

        val windTunnelView = findViewById<WindTunnelView>(R.id.windTunnelView)
        val btnReset = findViewById<Button>(R.id.btnReset)
        val btnSettings = findViewById<Button>(R.id.btnSettings)
        val btnGraph = findViewById<Button>(R.id.btnGraph)
        
        val graphContainer = findViewById<android.view.ViewGroup>(R.id.graphContainer)
        val forceGraphView = findViewById<ForceGraphView>(R.id.forceGraphView)
        val btnCloseGraph = findViewById<Button>(R.id.btnCloseGraph)

        btnReset.setOnClickListener {
            windTunnelView.reset()
        }

        btnSettings.setOnClickListener {
            showSettingsMenu(windTunnelView)
        }
        
        btnGraph.setOnClickListener {
            graphContainer.visibility = android.view.View.VISIBLE
            updateGraphLoop(windTunnelView, forceGraphView, graphContainer)
        }

        btnCloseGraph.setOnClickListener {
            graphContainer.visibility = android.view.View.GONE
        }

        // Go Fullscreen
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, window.decorView).let { controller ->
            controller.hide(WindowInsetsCompat.Type.systemBars())
            controller.systemBarsBehavior =
                WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
    }

    private fun showSettingsMenu(windTunnelView: WindTunnelView) {
        val dialog = BottomSheetDialog(this)
        val view = LayoutInflater.from(this).inflate(R.layout.settings_menu, null)
        dialog.setContentView(view)

        val rgSize = view.findViewById<RadioGroup>(R.id.rgSize)
        val sbSpeed = view.findViewById<SeekBar>(R.id.sbSpeed)
        val tvSpeedValue = view.findViewById<TextView>(R.id.tvSpeedValue)
        val sbDensity = view.findViewById<SeekBar>(R.id.sbDensity)
        val tvDensityValue = view.findViewById<TextView>(R.id.tvDensityValue)
        val sbViscosity = view.findViewById<SeekBar>(R.id.sbViscosity)
        val tvViscosityValue = view.findViewById<TextView>(R.id.tvViscosityValue)
        val rgShape = view.findViewById<RadioGroup>(R.id.rgShape)
        val sbBrushSize = view.findViewById<SeekBar>(R.id.sbBrushSize)
        val tvBrushSizeValue = view.findViewById<TextView>(R.id.tvBrushSizeValue)
        val rgViz = view.findViewById<RadioGroup>(R.id.rgViz)
        val swTelemetry = view.findViewById<SwitchCompat>(R.id.swTelemetry)
        val swAbsolute = view.findViewById<SwitchCompat>(R.id.swAbsolute)

        // Initialize with current values
        val currentSpeed = windTunnelView.getAirflowSpeed()
        sbSpeed.progress = (currentSpeed * 4.0f).toInt()
        tvSpeedValue.text = String.format(Locale.US, "%.1f m/s", currentSpeed)

        val currentDensity = windTunnelView.getDensity()
        sbDensity.progress = ((currentDensity - 0.5f) * 100.0f).toInt()
        tvDensityValue.text = String.format(Locale.US, "%.2f kg/m³", currentDensity)

        val currentViscosity = windTunnelView.getViscosity()
        sbViscosity.progress = (((currentViscosity - 1e-6f) / 9.9e-5f) * 100.0f).toInt()
        tvViscosityValue.text = String.format(Locale.US, "%.1e m²/s", currentViscosity)
        
        sbBrushSize.progress = windTunnelView.baseBrushSize
        tvBrushSizeValue.text = String.format(Locale.US, "%d px", windTunnelView.baseBrushSize)
        
        when (windTunnelView.visualizationMode) {
            NativeLBMEngine.VIZ_VELOCITY -> rgViz.check(R.id.rbVizVelocity)
            NativeLBMEngine.VIZ_PRESSURE -> rgViz.check(R.id.rbVizPressure)
            NativeLBMEngine.VIZ_TOTAL_PRESSURE -> rgViz.check(R.id.rbVizTotal)
        }

        swAbsolute.isChecked = windTunnelView.useAbsolutePressure
        swAbsolute.setOnCheckedChangeListener { _, isChecked ->
            windTunnelView.useAbsolutePressure = isChecked
        }

        rgSize.setOnCheckedChangeListener { _, checkedId ->
            when (checkedId) {
                R.id.rbSmall -> windTunnelView.reinit(200, 100)
                R.id.rbMedium -> windTunnelView.reinit(400, 200)
                R.id.rbLarge -> windTunnelView.reinit(600, 300)
            }
        }

        sbSpeed.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val velocity = progress / 4.0f
                windTunnelView.setAirflowSpeed(velocity)
                tvSpeedValue.text = String.format(Locale.US, "%.1f m/s", velocity)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        sbDensity.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                // Density from 0.5 to 2.5 (progress 0-200)
                val density = 0.5f + (progress / 100.0f)
                windTunnelView.setDensity(density)
                tvDensityValue.text = String.format(Locale.US, "%.2f kg/m³", density)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        sbViscosity.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                // Viscosity from 1e-6 to 1e-4
                val viscosity = 1e-6f + (progress / 100.0f) * 9.9e-5f
                windTunnelView.setViscosity(viscosity)
                tvViscosityValue.text = String.format(Locale.US, "%.1e m²/s", viscosity)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        rgShape.setOnCheckedChangeListener { _, checkedId ->
            when (checkedId) {
                R.id.rbCircle -> windTunnelView.brushShape = WindTunnelView.BrushShape.CIRCLE
                R.id.rbSquare -> windTunnelView.brushShape = WindTunnelView.BrushShape.SQUARE
            }
        }

        sbBrushSize.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                windTunnelView.baseBrushSize = progress.coerceAtLeast(1)
                tvBrushSizeValue.text = String.format(Locale.US, "%d px", progress.coerceAtLeast(1))
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        rgViz.setOnCheckedChangeListener { _, checkedId ->
            val mode = when (checkedId) {
                R.id.rbVizVelocity -> NativeLBMEngine.VIZ_VELOCITY
                R.id.rbVizPressure -> NativeLBMEngine.VIZ_PRESSURE
                R.id.rbVizTotal -> NativeLBMEngine.VIZ_TOTAL_PRESSURE
                else -> NativeLBMEngine.VIZ_VELOCITY
            }
            windTunnelView.updateVisualizationMode(mode)
        }

        swTelemetry.isChecked = windTunnelView.showDetailedTelemetry
        swTelemetry.setOnCheckedChangeListener { _, isChecked ->
            windTunnelView.showDetailedTelemetry = isChecked
        }

        dialog.show()
    }

    private fun updateGraphLoop(
        windTunnelView: WindTunnelView,
        forceGraphView: ForceGraphView,
        container: android.view.View
    ) {
        if (container.visibility == android.view.View.VISIBLE) {
            forceGraphView.updateData(windTunnelView.getForceHistory())
            container.postDelayed({
                updateGraphLoop(windTunnelView, forceGraphView, container)
            }, 100) // Update every 100ms
        }
    }
}