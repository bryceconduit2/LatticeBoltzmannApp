package com.example.latticeboltzmann

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        supportActionBar?.hide()
        setContentView(R.layout.activity_main)

        val windTunnelView = findViewById<WindTunnelView>(R.id.windTunnelView)
        val btnReset = findViewById<android.widget.Button>(R.id.btnReset)

        btnReset.setOnClickListener {
            windTunnelView.reset()
        }

        val sbAirflowSpeed = findViewById<android.widget.SeekBar>(R.id.sbAirflowSpeed)
        sbAirflowSpeed.setOnSeekBarChangeListener(object : android.widget.SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: android.widget.SeekBar?, progress: Int, fromUser: Boolean) {
                // Map progress (0-200) to physical velocity (0.0 to 50.0 m/s)
                val velocity = progress / 4.0f
                windTunnelView.setAirflowSpeed(velocity)
            }

            override fun onStartTrackingTouch(seekBar: android.widget.SeekBar?) {}
            override fun onStopTrackingTouch(seekBar: android.widget.SeekBar?) {}
        })

        // Go Fullscreen
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, window.decorView).let { controller ->
            controller.hide(WindowInsetsCompat.Type.systemBars())
            controller.systemBarsBehavior =
                WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
    }
}