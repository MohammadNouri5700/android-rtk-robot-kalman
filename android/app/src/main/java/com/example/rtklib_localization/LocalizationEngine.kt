package com.example.rtklib_localization

import android.os.SystemClock
import androidx.annotation.Keep
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.concurrent.atomic.AtomicBoolean


class LocalizationEngine : AutoCloseable {

    private var nativeEnginePtr: Long = 0L
    private val isInitialized = AtomicBoolean(false)

    // Modern StateFlow so Android UI can react to state changes automatically
    private val _engineState = MutableStateFlow<StateVector?>(null)
    val engineState: StateFlow<StateVector?> = _engineState.asStateFlow()

    companion object {
        init {
            // This name MUST match the shared library name in CMakeLists.txt!
            System.loadLibrary("localization_engine")
        }
    }

    init {
        nativeEnginePtr = nativeCreateEngine()
        isInitialized.set(true)
    }

    @Synchronized
    fun pushIMU(linearAccel: FloatArray, angularVelocity: FloatArray) {
        if (!isInitialized.get() || nativeEnginePtr == 0L) return

        val timestamp = SystemClock.elapsedRealtimeNanos() / 1000000000.0

        // 1. Pass the data down into the C++ Engine
        nativePushIMU(
            nativeEnginePtr, timestamp,
            linearAccel[0].toDouble(), linearAccel[1].toDouble(), linearAccel[2].toDouble(),
            angularVelocity[0].toDouble(), angularVelocity[1].toDouble(), angularVelocity[2].toDouble()
        )

        // 2. Retrieve the newly calculated 15-dimensional state from C++
        nativeGetState(nativeEnginePtr)?.let { array ->
            if (array.size == 15) {
                // 3. Map it to a clean Kotlin Data Class for the developer
                _engineState.value = StateVector(
                    x = array[0], y = array[1], z = array[2],
                    roll = array[3], pitch = array[4], yaw = array[5],
                    vx = array[6], vy = array[7], vz = array[8]
                )
            }
        }
    }

    @Synchronized
    override fun close() {
        if (isInitialized.compareAndSet(true, false)) {
            if (nativeEnginePtr != 0L) {
                nativeDestroyEngine(nativeEnginePtr)
                nativeEnginePtr = 0L
            }
        }
    }

    @Synchronized
    fun pushGNSS(latitude: Double, longitude: Double, altitude: Double, accuracy: Double) {
        if (!isInitialized.get() || nativeEnginePtr == 0L) return

        val timestamp = SystemClock.elapsedRealtimeNanos() / 1000000000.0

        // Push to C++ (squaring the accuracy to approximate variance/covariance)
        nativePushGNSS(
            nativeEnginePtr, timestamp,
            latitude, longitude, altitude,
            accuracy * accuracy, accuracy * accuracy, accuracy * accuracy
        )

        // Retrieve the updated state after the GNSS correction
        nativeGetState(nativeEnginePtr)?.let { array ->
            if (array.size == 15) {
                _engineState.value = StateVector(
                    x = array[0], y = array[1], z = array[2],
                    roll = array[3], pitch = array[4], yaw = array[5],
                    vx = array[6], vy = array[7], vz = array[8]
                )
            }
        }
    }

    // ... (Add this to the bottom with your other external functions) ...
    private external fun nativePushGNSS(ptr: Long, ts: Double, lat: Double, lon: Double, alt: Double, cx: Double, cy: Double, cz: Double)
    private external fun nativeCreateEngine(): Long
    private external fun nativeDestroyEngine(ptr: Long)
    private external fun nativePushIMU(ptr: Long, ts: Double, ax: Double, ay: Double, az: Double, wx: Double, wy: Double, wz: Double)
    private external fun nativeGetState(ptr: Long): DoubleArray?


}

// Clean Kotlin Data Class representing the C++ Eigen::VectorXd
@Keep
data class StateVector(
    val x: Double, val y: Double, val z: Double,
    val roll: Double, val pitch: Double, val yaw: Double,
    val vx: Double, val vy: Double, val vz: Double
)