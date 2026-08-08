package com.bc.fluidsandbox

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

/**
 * Main Controller for the Fluid Sandbox Application.
 * 
 * This activity handles high-level UI orchestration, connects the simulation view
 * to the settings menu, and manages the real-time telemetry graph logic.
 */
class MainActivity : AppCompatActivity() {
    
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        supportActionBar?.hide() // Immersive aesthetic
        setContentView(R.layout.activity_main)

        // --- View Identifiers ---
        val windTunnelView = findViewById<WindTunnelView>(R.id.windTunnelView)
        val btnReset = findViewById<Button>(R.id.btnReset)
        val btnSettings = findViewById<Button>(R.id.btnSettings)
        val btnGraph = findViewById<Button>(R.id.btnGraph)
        val btnPause = findViewById<Button>(R.id.btnPause)
        
        val graphContainer = findViewById<android.view.ViewGroup>(R.id.graphContainer)
        val forceGraphView = findViewById<ForceGraphView>(R.id.forceGraphView)
        val btnCloseGraph = findViewById<Button>(R.id.btnCloseGraph)
        val rgGraphMode = findViewById<RadioGroup>(R.id.rgGraphMode)

        // --- Simulation Controls ---
        btnReset.setOnClickListener {
            windTunnelView.reset() // Clears obstacles and wakes
        }

        btnPause.setOnClickListener {
            windTunnelView.isSimulationRunning = !windTunnelView.isSimulationRunning
            btnPause.text = if (windTunnelView.isSimulationRunning) {
                getString(R.string.btn_pause)
            } else {
                getString(R.string.btn_play)
            }
        }

        btnSettings.setOnClickListener {
            showSettingsMenu(windTunnelView) // Launch configuration menu
        }
        
        btnGraph.setOnClickListener {
            graphContainer.visibility = android.view.View.VISIBLE
            updateGraphLoop(windTunnelView, forceGraphView, graphContainer) // Start data stream
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

        // --- IMMERSIVE SYSTEM UI CONFIGURATION ---
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, window.decorView).let { controller ->
            controller.hide(WindowInsetsCompat.Type.systemBars()) // Hide nav and status bars
            controller.systemBarsBehavior =
                WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
    }

    /**
     * Inflates and manages the BottomSheet configuration menu.
     * Connects all SeekBars and Switches to the underlying WindTunnelView.
     */
    private fun showSettingsMenu(windTunnelView: WindTunnelView) {
        val dialog = BottomSheetDialog(this)
        val view = LayoutInflater.from(this).inflate(R.layout.settings_menu, null)
        dialog.setContentView(view)

        // Find all interactive elements in the menu
        val sbResolution = view.findViewById<SeekBar>(R.id.sbResolution)
        val tvResolutionValue = view.findViewById<TextView>(R.id.tvResolutionValue)
        val sbSpeed = view.findViewById<SeekBar>(R.id.sbSpeed)
        val tvSpeedValue = view.findViewById<TextView>(R.id.tvSpeedValue)
        val sbDX = view.findViewById<SeekBar>(R.id.sbDX)
        val tvDXValue = view.findViewById<TextView>(R.id.tvDXValue)
        val sbDensity = view.findViewById<SeekBar>(R.id.sbDensity)
        val tvDensityValue = view.findViewById<TextView>(R.id.tvDensityValue)
        val sbViscosity = view.findViewById<SeekBar>(R.id.sbViscosity)
        val tvViscosityValue = view.findViewById<TextView>(R.id.tvViscosityValue)
        val sbDtScale = view.findViewById<SeekBar>(R.id.sbDtScale)
        val tvDtValue = view.findViewById<TextView>(R.id.tvDtValue)
        val rgShape = view.findViewById<RadioGroup>(R.id.rgShape)
        val sbBrushSize = view.findViewById<SeekBar>(R.id.sbBrushSize)
        val tvBrushSizeValue = view.findViewById<TextView>(R.id.tvBrushSizeValue)
        val rgViz = view.findViewById<RadioGroup>(R.id.rgViz)
        val rgBoundary = view.findViewById<RadioGroup>(R.id.rgBoundary)
        val rgColorScheme = view.findViewById<RadioGroup>(R.id.rgColorScheme)
        val swTelemetry = view.findViewById<SwitchCompat>(R.id.swTelemetry)
        val swSmoothHD = view.findViewById<SwitchCompat>(R.id.swSmoothHD)
        val swAbsolute = view.findViewById<SwitchCompat>(R.id.swAbsolute)
        val swGridlines = view.findViewById<SwitchCompat>(R.id.swGridlines)
        val sbAoA = view.findViewById<SeekBar>(R.id.sbAoA)
        val tvAoAValue = view.findViewById<TextView>(R.id.tvAoAValue)
        val sbCores = view.findViewById<SeekBar>(R.id.sbCores)
        val tvCoreValue = view.findViewById<TextView>(R.id.tvCoreValue)
        val btnResetDefaults = view.findViewById<Button>(R.id.btnResetDefaults)

        // Dynamic Resolution Max: Set the slider limit based on the actual screen width
        // while maintaining a minimum of 160.
        val displayMetrics = resources.displayMetrics
        val screenWidth = maxOf(displayMetrics.widthPixels, displayMetrics.heightPixels)
        val maxSimWidth = screenWidth.coerceIn(160, 2000) // Cap at 2000 for memory safety
        sbResolution.max = maxSimWidth - 160

        val llNacaSelector = view.findViewById<android.view.View>(R.id.llNacaSelector)
        val spNacaProfiles = view.findViewById<android.widget.Spinner>(R.id.spNacaProfiles)

        // --- PROFILE DROPDOWN SETUP ---
        val nacaProfiles = arrayOf(
            "NACA 0012 (Symmetric)",
            "NACA 4412 (High Camber)",
            "NACA 2412 (Classic GA)",
            "NACA 0015 (Thick Symmetric)",
            "NACA 6412 (Extreme Lift)"
        )
        val adapter = android.widget.ArrayAdapter(this, R.layout.spinner_item, nacaProfiles)
        adapter.setDropDownViewResource(R.layout.spinner_item)
        spNacaProfiles.adapter = adapter

        // Logic to detect which profile is currently active in the engine
        val currentProfileIdx = when {
            windTunnelView.nacaM == 0.0f && windTunnelView.nacaT == 0.12f -> 0
            windTunnelView.nacaM == 0.04f && windTunnelView.nacaP == 0.4f -> 1
            windTunnelView.nacaM == 0.02f && windTunnelView.nacaP == 0.4f -> 2
            windTunnelView.nacaM == 0.0f && windTunnelView.nacaT == 0.15f -> 3
            windTunnelView.nacaM == 0.06f && windTunnelView.nacaP == 0.4f -> 4
            else -> 0
        }
        spNacaProfiles.setSelection(currentProfileIdx)

        // --- INITIAL STATE LOADING ---
        val currentSpeed = windTunnelView.getAirflowSpeed()
        sbSpeed.progress = (currentSpeed * 4.0f).toInt()
        tvSpeedValue.text = String.format(Locale.US, "%.1f m/s", currentSpeed)

        val currentDensity = windTunnelView.getDensity()
        sbDensity.progress = ((currentDensity - 0.5f) * 100.0f).toInt()
        tvDensityValue.text = String.format(Locale.US, "%.2f kg/m³", currentDensity)

        val currentDX = windTunnelView.currentSimulationDX
        sbDX.progress = ((currentDX - 0.0005f) / 0.0001f).toInt()
        tvDXValue.text = String.format(Locale.US, "%.1f mm", currentDX * 1000f)

        val currentViscosity = windTunnelView.getViscosity()
        sbViscosity.progress = (((currentViscosity - 1e-6f) / 9.9e-5f) * 100.0f).toInt()
        tvViscosityValue.text = String.format(Locale.US, "%.1e m²/s", currentViscosity)

        sbDtScale.progress = ((windTunnelView.simulationDtScale - 0.5f) * 10.0f).toInt().coerceIn(0, 45)
        tvDtValue.text = if (windTunnelView.simulationDtScale <= 1.05f) "Precise (%.1fx)".format(windTunnelView.simulationDtScale) 
                         else "Fast (%.1fx)".format(windTunnelView.simulationDtScale)

        sbBrushSize.progress = windTunnelView.baseBrushSize
        tvBrushSizeValue.text = String.format(Locale.US, "%d px", windTunnelView.baseBrushSize)

        sbAoA.progress = (windTunnelView.airfoilAoA + 20).toInt()
        tvAoAValue.text = String.format(Locale.US, "%.0f°", windTunnelView.airfoilAoA)
        
        // Match RadioButtons to current engine state
        when (windTunnelView.visualizationMode) {
            NativeLBMEngine.VIZ_VELOCITY -> rgViz.check(R.id.rbVizVelocity)
            NativeLBMEngine.VIZ_PRESSURE -> rgViz.check(R.id.rbVizPressure)
            NativeLBMEngine.VIZ_TOTAL_PRESSURE -> rgViz.check(R.id.rbVizTotal)
        }

        when (windTunnelView.boundaryMode) {
            NativeLBMEngine.BND_PERIODIC -> rgBoundary.check(R.id.rbBndPeriodic)
            NativeLBMEngine.BND_OPEN -> rgBoundary.check(R.id.rbBndOpen)
        }

        when (windTunnelView.colorScheme) {
            0 -> rgColorScheme.check(R.id.rbColorStandard)
            1 -> rgColorScheme.check(R.id.rbColorIronbow)
            2 -> rgColorScheme.check(R.id.rbColorGrayscale)
            3 -> rgColorScheme.check(R.id.rbColorOcean)
        }

        when (windTunnelView.brushShape) {
            WindTunnelView.BrushShape.CIRCLE -> rgShape.check(R.id.rbCircle)
            WindTunnelView.BrushShape.SQUARE -> rgShape.check(R.id.rbSquare)
            WindTunnelView.BrushShape.NACA -> {
                rgShape.check(R.id.rbNaca)
                llNacaSelector.visibility = android.view.View.VISIBLE
            }
        }

        val currentSimWidth = windTunnelView.getSimWidth()
        val currentSimHeight = windTunnelView.getSimHeight()
        sbResolution.progress = (currentSimWidth - 160).coerceIn(0, sbResolution.max)
        tvResolutionValue.text = String.format(Locale.US, "%dx%d", currentSimWidth, currentSimHeight)

        // --- MENU INTERACTION LISTENERS ---

        swAbsolute.isChecked = windTunnelView.useAbsolutePressure
        swAbsolute.setOnCheckedChangeListener { _, isChecked ->
            windTunnelView.useAbsolutePressure = isChecked
        }

        rgBoundary.setOnCheckedChangeListener { _, checkedId ->
            val mode = when (checkedId) {
                R.id.rbBndPeriodic -> NativeLBMEngine.BND_PERIODIC
                R.id.rbBndOpen -> NativeLBMEngine.BND_OPEN
                else -> NativeLBMEngine.BND_PERIODIC
            }
            windTunnelView.updateBoundaryMode(mode)
        }

        rgColorScheme.setOnCheckedChangeListener { _, checkedId ->
            val scheme = when (checkedId) {
                R.id.rbColorStandard -> 0
                R.id.rbColorIronbow -> 1
                R.id.rbColorGrayscale -> 2
                R.id.rbColorOcean -> 3
                else -> 0
            }
            windTunnelView.updateColorScheme(scheme)
        }

        sbResolution.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val w = progress + 160
                val h = w / 2
                tvResolutionValue.text = String.format(Locale.US, "%dx%d", w, h)
                if (fromUser) windTunnelView.reinit(w, h)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        sbSpeed.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val velocity = progress / 4.0f
                windTunnelView.setAirflowSpeed(velocity)
                tvSpeedValue.text = String.format(Locale.US, "%.1f m/s", velocity)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        sbDX.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val dx = 0.0005f + (progress * 0.0001f)
                windTunnelView.setSimulationDX(dx)
                tvDXValue.text = String.format(Locale.US, "%.1f mm", dx * 1000f)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        sbDensity.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val density = 0.5f + (progress / 100.0f)
                windTunnelView.setDensity(density)
                tvDensityValue.text = String.format(Locale.US, "%.2f kg/m³", density)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        sbViscosity.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val viscosity = 1e-6f + (progress / 100.0f) * 9.9e-5f
                windTunnelView.setViscosity(viscosity)
                tvViscosityValue.text = String.format(Locale.US, "%.1e m²/s", viscosity)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        sbDtScale.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                val scale = 0.5f + (progress / 10.0f)
                windTunnelView.updateDtScale(scale)
                tvDtValue.text = if (scale <= 1.05f) "Precise (%.1fx)".format(scale) 
                                 else "Fast (%.1fx)".format(scale)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: SeekBar?) {}
        })

        rgShape.setOnCheckedChangeListener { _, checkedId ->
            when (checkedId) {
                R.id.rbCircle -> {
                    windTunnelView.brushShape = WindTunnelView.BrushShape.CIRCLE
                    llNacaSelector.visibility = android.view.View.GONE
                }
                R.id.rbSquare -> {
                    windTunnelView.brushShape = WindTunnelView.BrushShape.SQUARE
                    llNacaSelector.visibility = android.view.View.GONE
                }
                R.id.rbNaca -> {
                    windTunnelView.brushShape = WindTunnelView.BrushShape.NACA
                    llNacaSelector.visibility = android.view.View.VISIBLE
                }
            }
        }

        spNacaProfiles.onItemSelectedListener = object : android.widget.AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: android.widget.AdapterView<*>?, view: android.view.View?, position: Int, id: Long) {
                when (position) {
                    0 -> { windTunnelView.nacaM = 0.0f; windTunnelView.nacaP = 0.0f; windTunnelView.nacaT = 0.12f } 
                    1 -> { windTunnelView.nacaM = 0.04f; windTunnelView.nacaP = 0.4f; windTunnelView.nacaT = 0.12f }
                    2 -> { windTunnelView.nacaM = 0.02f; windTunnelView.nacaP = 0.4f; windTunnelView.nacaT = 0.12f } 
                    3 -> { windTunnelView.nacaM = 0.0f; windTunnelView.nacaP = 0.0f; windTunnelView.nacaT = 0.15f }
                    4 -> { windTunnelView.nacaM = 0.06f; windTunnelView.nacaP = 0.4f; windTunnelView.nacaT = 0.12f }
                }
            }
            override fun onNothingSelected(parent: android.widget.AdapterView<*>?) {}
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

        swSmoothHD.isChecked = windTunnelView.useSmoothHD
        swSmoothHD.setOnCheckedChangeListener { _, isChecked ->
            windTunnelView.useSmoothHD = isChecked
        }

        swGridlines.isChecked = windTunnelView.showGridlines
        swGridlines.setOnCheckedChangeListener { _, isChecked ->
            windTunnelView.showGridlines = isChecked
        }

        val maxCores = windTunnelView.getMaxAvailableCores()
        sbCores.max = maxCores - 1 
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

        btnResetDefaults.setOnClickListener {
            // 1. Reset Internal State
            windTunnelView.reinit(400, 200)
            windTunnelView.setAirflowSpeed(30.0f)
            windTunnelView.setSimulationDX(0.0025f)
            windTunnelView.setDensity(1.225f)
            windTunnelView.setViscosity(1.5e-5f)
            windTunnelView.updateDtScale(1.0f)
            windTunnelView.brushShape = WindTunnelView.BrushShape.CIRCLE
            windTunnelView.baseBrushSize = 8
            windTunnelView.airfoilAoA = 0.0f
            windTunnelView.updateVisualizationMode(NativeLBMEngine.VIZ_VELOCITY)
            windTunnelView.updateBoundaryMode(NativeLBMEngine.BND_PERIODIC)
            windTunnelView.updateColorScheme(0)
            windTunnelView.useAbsolutePressure = false
            windTunnelView.showDetailedTelemetry = true
            windTunnelView.useSmoothHD = false
            windTunnelView.showGridlines = false
            windTunnelView.updateCoreCount(4.coerceAtMost(maxCores))

            // 2. Update UI Widgets in the menu
            sbResolution.progress = 400 - 160
            tvResolutionValue.text = String.format(Locale.US, "400x200")
            sbSpeed.progress = (30.0f * 4.0f).toInt()
            tvSpeedValue.text = String.format(Locale.US, "%.1f m/s", 30.0f)
            sbDX.progress = ((0.0025f - 0.0005f) / 0.0001f).toInt()
            tvDXValue.text = String.format(Locale.US, "2.5 mm")
            sbDensity.progress = ((1.225f - 0.5f) * 100.0f).toInt()
            tvDensityValue.text = String.format(Locale.US, "%.2f kg/m³", 1.225f)
            sbViscosity.progress = (((1.5e-5f - 1e-6f) / 9.9e-5f) * 100.0f).toInt()
            tvViscosityValue.text = String.format(Locale.US, "%.1e m²/s", 1.5e-5f)
            sbDtScale.progress = ((1.0f - 0.5f) * 10.0f).toInt()
            tvDtValue.text = "Precise (1.0x)"
            rgShape.check(R.id.rbCircle)
            llNacaSelector.visibility = android.view.View.GONE
            sbBrushSize.progress = 8
            tvBrushSizeValue.text = String.format(Locale.US, "%d px", 8)
            sbAoA.progress = 20
            tvAoAValue.text = String.format(Locale.US, "%.0f°", 0.0f)
            rgViz.check(R.id.rbVizVelocity)
            swAbsolute.isChecked = false
            rgBoundary.check(R.id.rbBndPeriodic)
            rgColorScheme.check(R.id.rbColorStandard)
            swTelemetry.isChecked = true
            swSmoothHD.isChecked = false
            swGridlines.isChecked = false
            sbCores.progress = (4.coerceAtMost(maxCores)) - 1
            tvCoreValue.text = (4.coerceAtMost(maxCores)).toString()
        }

        // Expand BottomSheet to full height automatically
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

    /**
     * Background data sampler that periodically pulls telemetry from the engine
     * and pushes it to the ForceGraphView for rendering.
     */
    private fun updateGraphLoop(
        windTunnelView: WindTunnelView,
        forceGraphView: ForceGraphView,
        container: android.view.View
    ) {
        if (container.visibility == android.view.View.VISIBLE) {
            forceGraphView.updateData(windTunnelView.getForceHistory())
            container.postDelayed({
                updateGraphLoop(windTunnelView, forceGraphView, container)
            }, 100) // Update every 10 frames (100ms)
        }
    }
}
