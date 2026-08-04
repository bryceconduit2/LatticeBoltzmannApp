package com.bc.fluidsandbox

import androidx.test.platform.app.InstrumentationRegistry
import androidx.test.ext.junit.runners.AndroidJUnit4

import org.junit.Test
import org.junit.runner.RunWith

import org.junit.Assert.*

/**
 * Basic Integration Test for Fluid Sandbox.
 * 
 * This ensures that the Android system is providing the correct package context
 * and that the basic application environment is stable.
 */
@RunWith(AndroidJUnit4::class)
class ExampleInstrumentedTest {
    @Test
    fun useAppContext() {
        // Context of the app under test.
        val appContext = InstrumentationRegistry.getInstrumentation().targetContext
        assertEquals("com.bc.fluidsandbox", appContext.packageName)
    }
}
