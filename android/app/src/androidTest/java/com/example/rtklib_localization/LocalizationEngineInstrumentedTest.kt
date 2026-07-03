package com.example.rtklib_localization

import androidx.test.ext.junit.runners.AndroidJUnit4
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith

/**
 * Instrumented test, which will execute on an Android device.
 *
 * This test verifies the JNI integration and the EKF logic by running
 * the native code on the target device's architecture.
 */
@RunWith(AndroidJUnit4::class)
class LocalizationEngineInstrumentedTest {

    @Test
    fun testEngineIntegration() {
        // Initialize the engine (loads native library in companion object)
        val engine = LocalizationEngine()
        assertNotNull("Engine should be initialized", engine)

        // 1. Initial GNSS Update (at origin)
        // Accuracy 0.1m
        engine.pushGNSS(0.0, 0.0, 0.0, 0.1)
        
        val initialState = engine.engineState.value
        assertNotNull("State should not be null after GNSS update", initialState)
        // We expect position to be roughly at origin (lat=0, lon=0 -> x=0, y=0 in ENU usually)
        // Note: The EKF internal coordinate system depends on the first GNSS fix.
        
        // 2. Push IMU data (constant acceleration in X)
        // We push several samples to allow the filter to integrate
        for (i in 1..5) {
            engine.pushIMU(
                linearAccel = floatArrayOf(2.0f, 0.0f, 0.0f),
                angularVelocity = floatArrayOf(0.0f, 0.0f, 0.0f)
            )
            Thread.sleep(10) // Small delay to simulate real-time
        }

        val updatedState = engine.engineState.value
        assertNotNull("State should be updated after IMU", updatedState)
        
        // 3. Verify that the state has changed from the initial state
        // After pushing acceleration in X, the velocity vx or position x should have increased.
        assertTrue("Velocity vx should have increased from zero", updatedState!!.vx > 0.0)
        assertTrue("Position x should have increased from zero", updatedState.x > 0.0)
        
        engine.close()
    }
}
