package org.firstinspires.ftc.teamcode.variants

import android.util.Log
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.TurretImpl
import kotlin.collections.plusAssign
import kotlin.compareTo
import kotlin.div
import kotlin.math.abs
import kotlin.text.compareTo
import kotlin.times
import kotlin.unaryMinus

class TurretImpl2 (private val hw: CompBot2Hardware){

    companion object{
        const val TICKS_PER_DEGREE = 159.5
        const val POSITIVE_LIMIT_DEG = 135.0
        const val POSITIVE_LIMIT_TICK = POSITIVE_LIMIT_DEG * TICKS_PER_DEGREE
        const val NEGATIVE_LIMIT_DEG = -135.0
        const val NEGATIVE_LIMIT_TICK = NEGATIVE_LIMIT_DEG * TICKS_PER_DEGREE
        const val DEADBAND_TICKS = 159.5 / 2.0  // 1 deg
        const val I_SPEED_LIMIT = 5_000.0 * 2.33
        const val SLEW_RATE_LIMITER =
            0.3 // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html

        // approx min power 0.0486

        private var INTEGRAL_ERROR_SUM_LIMIT = 220.0 * 2 * 2.33 // DEPENDS ON kI
        var P = 0.000_10
        //        private var I = 0.000_2 / 2.33 // 0.000_2
        var I = 0.088 / INTEGRAL_ERROR_SUM_LIMIT // 0.000_2
        // current best options: 1e-5, 5e-6
        var D = 0.000_005 // 0.000_062

    }


    private var resetPid = true
    var targetAngleDeg = 0.0
    private var lastPidTime = 0L
    private var prevError = 0.0
    private var integralErrorSum = 0.0
    private var prevOutput = 0.0
    private var maxIntegralErrorSum = 0.0


    fun tickTurret(){
        val targetTicks = -targetAngleDeg * TICKS_PER_DEGREE
        val currentPosition = hw.turretEncoder.getCurrentPosition()
        val error = targetTicks - currentPosition
        Log.i("Turret", "err %.2f to %.2f (at %d)".format(error, targetTicks, currentPosition))
        val now = System.nanoTime()
        var dt = 0.0
        if (lastPidTime != 0L) {
            dt = (now - lastPidTime) / 1e9
        }
        lastPidTime = now

        if (abs(error) <= DEADBAND_TICKS) {
            prevError = error
            integralErrorSum = 0.0
            hw.setTurretPower(0.0)
            return
        }

        if (resetPid) {
            prevError = error
            integralErrorSum = 0.0
            resetPid = false
        }

        if (error * prevError < 0.0) {
            integralErrorSum = 0.0
        }

        val derivative = if (dt > 0.0) (error - prevError) / dt else 0.0

        if (abs(derivative) <= I_SPEED_LIMIT) {
            val integralError = error * dt
            integralErrorSum += integralError

            integralErrorSum = when {
                integralErrorSum > INTEGRAL_ERROR_SUM_LIMIT -> INTEGRAL_ERROR_SUM_LIMIT
                integralErrorSum < -INTEGRAL_ERROR_SUM_LIMIT -> -INTEGRAL_ERROR_SUM_LIMIT
                else -> integralErrorSum
            }

//            Log.w(
//                "integralErrorSum (Current)",
//                ".2f".format(integralErrorSum)
//            )

            if (integralErrorSum > maxIntegralErrorSum) {
                maxIntegralErrorSum = integralErrorSum
            }

            if (-integralErrorSum > maxIntegralErrorSum) {
                maxIntegralErrorSum = -integralErrorSum
            }

//            Log.w(
//                "maxIntegralErrorSum",
//                "%.2f".format(maxIntegralErrorSum)
//            )
        } else {
            integralErrorSum = 0.0
        }

        prevError = error
        val output: Double = (P * error) + (I * integralErrorSum) + (D * derivative)

//        Log.i(
//            "TurretImpl",
//            "P %.2f I %.2f D %.2f => %.2f".format(
//                P * error,
//                I * integralErrorSum,
//                D * derivative,
//                output
//            )
//        )
        val output2 = when {
            output > 1.0 -> 1.0
            output < -1.0 -> -1.0
            else -> output
        }

        var output3 = when {
            output2 > prevOutput + SLEW_RATE_LIMITER -> prevOutput + SLEW_RATE_LIMITER
            output2 < prevOutput - SLEW_RATE_LIMITER -> prevOutput - SLEW_RATE_LIMITER
            else -> output2
        }

        prevOutput = output3

        if (currentPosition < NEGATIVE_LIMIT_TICK && output3 < 0) output3 = 0.0
        if (currentPosition > POSITIVE_LIMIT_TICK && output3 > 0) output3 = 0.0

        hw.setTurretPower(output3)

        return
    }

    fun setTurretDeltaTarget(angle: Double) {
        val currentAngleDeg = -hw.turretEncoder.getCurrentPosition() / TurretImpl.TICKS_PER_DEGREE
        setTurretTarget(currentAngleDeg + angle)
    }

    fun setTurretTarget(angle: Double) {
        targetAngleDeg = when {
            angle > TurretImpl.POSITIVE_LIMIT_DEG -> TurretImpl.POSITIVE_LIMIT_DEG
            angle < TurretImpl.NEGATIVE_LIMIT_DEG -> TurretImpl.NEGATIVE_LIMIT_DEG
            else -> angle
        }
        Log.w("Target Angle", "targetAngleDeg: angle %.4f".format(angle))
    }
    fun currentPosition() = hw.turretEncoder.getCurrentPosition()
    fun velocity() = hw.turretEncoder.getVelocity()
}

