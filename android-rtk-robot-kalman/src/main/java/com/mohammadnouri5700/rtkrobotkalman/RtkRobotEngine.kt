package com.mohammadnouri5700.rtkrobotkalman

import android.os.SystemClock
import androidx.annotation.Keep
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.concurrent.atomic.AtomicBoolean

/**
 * RtkRobotEngine is the primary interface for the RTK Robot Kalman filter.
 * It coordinates sensor data input (IMU, GNSS) and provides the fused state vector.
 */
class RtkRobotEngine : AutoCloseable {

    private var nativeEnginePtr: Long = 0L
    private val isInitialized = AtomicBoolean(false)

    // Pre-allocated buffer to avoid GC thrashing at 50-100Hz
    private val stateBuffer = DoubleArray(15)

    // Modern StateFlow so Android UI can react to state changes automatically
    private val _engineState = MutableStateFlow<StateVector?>(null)
    val engineState: StateFlow<StateVector?> = _engineState.asStateFlow()

    companion object {
        init {
            // This name MUST match the shared library name in CMakeLists.txt!
            System.loadLibrary("rtk_robot_kalman_engine")
        }
    }

    init {
        nativeEnginePtr = nativeCreateEngine()
        isInitialized.set(true)
    }

    @Synchronized
    fun pushIMU(timestampNanos: Long, linearAccel: FloatArray, angularVelocity: FloatArray) {
        if (!isInitialized.get() || nativeEnginePtr == 0L) return

        val timestamp = timestampNanos / 1_000_000_000.0

        // 1. Pass the data down into the C++ Engine
        nativePushIMU(
            nativeEnginePtr, timestamp,
            linearAccel[0].toDouble(), linearAccel[1].toDouble(), linearAccel[2].toDouble(),
            angularVelocity[0].toDouble(), angularVelocity[1].toDouble(), angularVelocity[2].toDouble()
        )

        // 2. Retrieve the newly calculated 15-dimensional state into pre-allocated buffer
        nativeGetState(nativeEnginePtr, stateBuffer)
        
        // 3. Update the state Flow
        _engineState.value = StateVector(
            latitude = stateBuffer[0], longitude = stateBuffer[1], altitude = stateBuffer[2],
            roll = stateBuffer[3], pitch = stateBuffer[4], yaw = stateBuffer[5],
            vx = stateBuffer[6], vy = stateBuffer[7], vz = stateBuffer[8]
        )
    }

    @Synchronized
    fun pushGNSS(
        timestampNanos: Long,
        latitude: Double,
        longitude: Double,
        altitude: Double,
        vEast: Double,
        vNorth: Double,
        vUp: Double,
        hasVelocity: Boolean,
        covariance: DoubleArray // 3x3 serialized array
    ) {
        if (!isInitialized.get() || nativeEnginePtr == 0L) return

        val timestamp = timestampNanos / 1_000_000_000.0

        nativePushGNSS(
            nativeEnginePtr, timestamp,
            latitude, longitude, altitude,
            vEast, vNorth, vUp, hasVelocity,
            covariance
        )

        // Retrieve the updated state after the GNSS correction
        nativeGetState(nativeEnginePtr, stateBuffer)
        
        _engineState.value = StateVector(
            latitude = stateBuffer[0], longitude = stateBuffer[1], altitude = stateBuffer[2],
            roll = stateBuffer[3], pitch = stateBuffer[4], yaw = stateBuffer[5],
            vx = stateBuffer[6], vy = stateBuffer[7], vz = stateBuffer[8]
        )
    }

    @Synchronized
    fun pushOrientation(timestampNanos: Long, roll: Double, pitch: Double, yaw: Double) {
        if (!isInitialized.get() || nativeEnginePtr == 0L) return
        val timestamp = timestampNanos / 1_000_000_000.0
        nativePushOrientation(nativeEnginePtr, timestamp, roll, pitch, yaw)

        // Retrieve the updated state after orientation push
        nativeGetState(nativeEnginePtr, stateBuffer)

        _engineState.value = StateVector(
            latitude = stateBuffer[0], longitude = stateBuffer[1], altitude = stateBuffer[2],
            roll = stateBuffer[3], pitch = stateBuffer[4], yaw = stateBuffer[5],
            vx = stateBuffer[6], vy = stateBuffer[7], vz = stateBuffer[8]
        )
    }

    @Synchronized
    fun pushRawObservations(timestampNanos: Long, observations: Array<DoubleArray>) {
        if (!isInitialized.get() || nativeEnginePtr == 0L) return
        val timestamp = timestampNanos / 1_000_000_000.0
        nativePushRawObservations(nativeEnginePtr, timestamp, observations)
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

    // Native Bridge Functions
    private external fun nativeCreateEngine(): Long
    private external fun nativeDestroyEngine(ptr: Long)
    private external fun nativePushIMU(ptr: Long, ts: Double, ax: Double, ay: Double, az: Double, wx: Double, wy: Double, wz: Double)
    private external fun nativePushGNSS(ptr: Long, ts: Double, lat: Double, lon: Double, alt: Double, vx: Double, vy: Double, vz: Double, hv: Boolean, cov: DoubleArray)
    private external fun nativePushOrientation(ptr: Long, ts: Double, roll: Double, pitch: Double, yaw: Double)
    private external fun nativePushRawObservations(ptr: Long, ts: Double, obs: Array<DoubleArray>)
    private external fun nativeGetState(ptr: Long, outArray: DoubleArray)
}

/**
 * Clean Kotlin Data Class representing the fused state.
 */
@Keep
data class StateVector(
    val latitude: Double, val longitude: Double, val altitude: Double,
    val roll: Double, val pitch: Double, val yaw: Double,
    val vx: Double, val vy: Double, val vz: Double
)
