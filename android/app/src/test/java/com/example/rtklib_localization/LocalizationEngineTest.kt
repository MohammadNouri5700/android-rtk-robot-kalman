package com.example.rtklib_localization

import org.junit.Assert.assertEquals
import org.junit.Test

class LocalizationEngineTest {

    @Test
    fun testStateVectorCreation() {
        val state = StateVector(
            x = 1.0, y = 2.0, z = 3.0,
            roll = 0.1, pitch = 0.2, yaw = 0.3,
            vx = 0.4, vy = 0.5, vz = 0.6
        )
        assertEquals(1.0, state.x, 0.0)
        assertEquals(0.3, state.yaw, 0.0)
        assertEquals(0.6, state.vz, 0.0)
    }

    // Note: native methods cannot be tested in a standard JVM unit test 
    // without loading the corresponding shared library for the host architecture.
}
