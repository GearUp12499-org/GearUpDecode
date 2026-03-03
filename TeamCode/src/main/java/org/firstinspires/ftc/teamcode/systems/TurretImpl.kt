package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import kotlin.math.abs

class TurretImpl(private val hw: CompBot2Hardware) : Task<TurretImpl>() {
    companion object {
        private val LOCK_ROOT = Lock.StrLock("turret_impl")
        const val TICKS_PER_DEGREE = 67.9
        const val POSITIVE_LIMIT_DEG = 135.0
        const val POSITIVE_LIMIT_TICK = POSITIVE_LIMIT_DEG * TICKS_PER_DEGREE
        const val NEGATIVE_LIMIT_DEG = -135.0
        const val NEGATIVE_LIMIT_TICK = NEGATIVE_LIMIT_DEG * TICKS_PER_DEGREE
        const val DEADBAND_TICKS = 136.0 / 2 // TODO: Try to revise these values
        const val I_SPEED_LIMIT = 5_000.0
        const val SLEW_RATE_LIMITER =
            0.3 // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
        private var P = 0.000_1
        private var I = 0.000_2 // 0.000_2
        private var D = 0.000_000 // 0.000_062
        private var INTEGRAL_ERROR_SUM_LIMIT = 220.0 * 2 // DEPENDS ON kI

        init {
            systemPackages.add(TurretImpl::class.qualifiedName!!)
        }
    }

    fun setPIDCoeffs(kP: Double, kI: Double, kD: Double, iLimit: Double) {
        P = kP
        I = kI
        D = kD
        INTEGRAL_ERROR_SUM_LIMIT = iLimit
    }

    fun setDeltaTarget(angle: Double) {
        val currentAngleDeg = -hw.turretEncoder.getCurrentPosition() / TICKS_PER_DEGREE
        setTarget(currentAngleDeg + angle)
    }

    fun setTarget(angle: Double) {
        targetAngleDeg = when {
            angle > POSITIVE_LIMIT_DEG -> POSITIVE_LIMIT_DEG
            angle < NEGATIVE_LIMIT_DEG -> NEGATIVE_LIMIT_DEG
            else -> angle
        }
    }

    fun getPower(): Double {
        return hw.turretPower
    }

    private var resetPid = true
    private var targetAngleDeg = 0.0
    private var lastPidTime = 0L
    private var prevError = 0.0
    private var integralErrorSum = 0.0
    private var prevOutput = 0.0
    private var maxIntegralErrorSum = 0.0

    val lock = LOCK_ROOT.derive()

    override fun onTick(): Boolean {
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
            return false
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

            Log.w(
                "integralErrorSum (Current)",
                ".2f".format(integralErrorSum)
            )

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

        Log.i(
            "TurretImpl",
            "P %.2f I %.2f D %.2f => %.2f".format(
                P * error,
                I * integralErrorSum,
                D * derivative,
                output
            )
        )
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

        return false
    }

    fun currentPosition() = hw.turretEncoder.getCurrentPosition()
    fun velocity() = hw.turretEncoder.getVelocity()
    fun setPower(power: Double) {
        hw.setTurretPower(power)
    }
}