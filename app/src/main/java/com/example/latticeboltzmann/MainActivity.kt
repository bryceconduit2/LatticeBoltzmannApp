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
        val rgGraphMode = findViewById<RadioGroup>(R.id.rgGraphMode)

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

        rgGraphMode.setOnCheckedChangeListener { _, checkedId ->
            forceGraphView.displayMode = if (checkedId == R.id.rbModeForce) {
                ForceGraphView.DisplayMode.FORCE
            } else {
                ForceGraphView.DisplayMode.COEFFICIENT
            }
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
        val rgBoundary = view.findViewById<RadioGroup>(R.id.rgBoundary)
        val swTelemetry = view.findViewById<SwitchCompat>(R.id.swTelemetry)
        val swAbsolute = view.findViewById<SwitchCompat>(R.id.swAbsolute)
        val swGridlines = view.findViewById<SwitchCompat>(R.id.swGridlines)
        val sbAoA = view.findViewById<SeekBar>(R.id.sbAoA)
        val tvAoAValue = view.findViewById<TextView>(R.id.tvAoAValue)
        val sbCores = view.findViewById<SeekBar>(R.id.sbCores)
        val tvCoreValue = view.findViewById<TextView>(R.id.tvCoreValue)

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

        sbAoA.progress = (windTunnelView.airfoilAoA + 20).toInt()
        tvAoAValue.text = String.format(Locale.US, "%.0f°", windTunnelView.airfoilAoA)
        
        when (windTunnelView.visualizationMode) {
            NativeLBMEngine.VIZ_VELOCITY -> rgViz.check(R.id.rbVizVelocity)
            NativeLBMEngine.VIZ_PRESSURE -> rgViz.check(R.id.rbVizPressure)
            NativeLBMEngine.VIZ_TOTAL_PRESSURE -> rgViz.check(R.id.rbVizTotal)
        }

        when (windTunnelView.boundaryMode) {
            NativeLBMEngine.BND_PERIODIC -> rgBoundary.check(R.id.rbBndPeriodic)
            NativeLBMEngine.BND_NO_SLIP -> rgBoundary.check(R.id.rbBndNoSlip)
            NativeLBMEngine.BND_FREE_SLIP -> rgBoundary.check(R.id.rbBndFreeSlip)
        }

        when (windTunnelView.brushShape) {
            WindTunnelView.BrushShape.CIRCLE -> rgShape.check(R.id.rbCircle)
            WindTunnelView.BrushShape.SQUARE -> rgShape.check(R.id.rbSquare)
            WindTunnelView.BrushShape.NACA0012 -> rgShape.check(R.id.rbNaca0012)
            WindTunnelView.BrushShape.NACA4412 -> rgShape.check(R.id.rbNaca4412)
        }

        // Initialize Tunnel Size selection based on current width
        // Small=200, Medium=400, Large=600
        val currentSimWidth = windTunnelView.getSimWidth()
        when (currentSimWidth) {
            160 -> rgSize.check(R.id.rbTiny)
            200 -> rgSize.check(R.id.rbSmall)
            400 -> rgSize.check(R.id.rbMedium)
            600 -> rgSize.check(R.id.rbLarge)
        }

        swAbsolute.isChecked = windTunnelView.useAbsolutePressure
        swAbsolute.setOnCheckedChangeListener { _, isChecked ->
            windTunnelView.useAbsolutePressure = isChecked
        }

        rgBoundary.setOnCheckedChangeListener { _, checkedId ->
            val mode = when (checkedId) {
                R.id.rbBndPeriodic -> NativeLBMEngine.BND_PERIODIC
                R.id.rbBndNoSlip -> NativeLBMEngine.BND_NO_SLIP
                R.id.rbBndFreeSlip -> NativeLBMEngine.BND_FREE_SLIP
                else -> NativeLBMEngine.BND_PERIODIC
            }
            windTunnelView.updateBoundaryMode(mode)
        }

        rgSize.setOnCheckedChangeListener { _, checkedId ->
            when (checkedId) {
                R.id.rbTiny -> windTunnelView.reinit(160, 80)
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
                R.id.rbNaca0012 -> windTunnelView.brushShape = WindTunnelView.BrushShape.NACA0012
                R.id.rbNaca4412 -> windTunnelView.brushShape = WindTunnelView.BrushShape.NACA4412
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

        sbAoA.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val aoa = progress - 20f
                windTunnelView.airfoilAoA = aoa
                tvAoAValue.text = String.format(Locale.US, "%.0f°", aoa)
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

        swGridlines.isChecked = windTunnelView.showGridlines
        swGridlines.setOnCheckedChangeListener { _, isChecked ->
            windTunnelView.showGridlines = isChecked
        }

        // Initialize Core slider
        val maxCores = windTunnelView.getMaxAvailableCores()
        sbCores.max = maxCores - 1 // 0-indexed internally, but we'll offset it by 1
        sbCores.progress = windTunnelView.coreCount - 1
        tvCoreValue.text = windTunnelView.coreCount.toString()

        sbCores.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val cores = progress + 1
                windTunnelView.updateCoreCount(cores)
                tvCoreValue.text = cores.toString()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        // Ensure the bottom sheet is fully expanded so the internal scroll view works predictably
        dialog.setOnShowListener {
            val bottomSheet = dialog.findViewById<android.view.View>(com.google.android.material.R.id.design_bottom_sheet)
            bottomSheet?.let {
                val behavior = com.google.android.material.bottomsheet.BottomSheetBehavior.from(it)
                behavior.state = com.google.android.material.bottomsheet.BottomSheetBehavior.STATE_EXPANDED
                behavior.skipCollapsed = true
            }
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