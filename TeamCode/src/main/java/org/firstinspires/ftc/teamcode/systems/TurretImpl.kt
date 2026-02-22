package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import kotlin.math.abs
import kotlin.math.sign

class TurretImpl(private val hw: CompBot2Hardware) : Task<TurretImpl>() {
    companion object {
        private val LOCK_ROOT = Lock.StrLock("turret_impl")
        const val TICKS_PER_DEGREE = 67.9
        const val POSITIVE_LIMIT = 9400.0
        const val NEGATIVE_LIMIT = -9400.0
        const val DEADBAND_TICKS = 136.0
        const val MIN_POWER_ERROR_TICKS = 320.0
        const val I_ZONE_TICKS = 650.0
        const val NEAR_TARGET_I_CLAMP = 2500.0
        const val MAX_I = 20_000.0

        const val P = 0.001
        const val I = 0.000_007
        const val D = 0.000_062

        const val BASE_POWER = 0.1

        init {
            systemPackages.add(TurretImpl::class.qualifiedName!!)
        }
    }

    fun setDeltaTarget(angle: Double) {
        setTarget(targetAngle + angle)
    }

    fun setTarget(angle: Double) {
        targetAngle = when {
            angle > POSITIVE_LIMIT -> POSITIVE_LIMIT
            angle < NEGATIVE_LIMIT -> NEGATIVE_LIMIT
            else -> angle
        }
    }

    private var resetPid = true
    private var targetAngle = 0.0
    private var lastPidTime = 0L
    private var prevError = 0.0
    private var integralError = 0.0

    val lock = LOCK_ROOT.derive()

    private var suspended = false

    override fun onTick(): Boolean {
        if (suspended) return false

        val targetTicks = -targetAngle * TICKS_PER_DEGREE
        val error = targetTicks - hw.turretEncoder.getCurrentPosition()
        val now = System.nanoTime()
        var dt = 0.0
        if (lastPidTime != 0L) {
            dt = (now - lastPidTime) / 1e9
        }
        lastPidTime = now

        if (abs(error) <= DEADBAND_TICKS) {
            prevError = error
            integralError = 0.0
            hw.setTurretPower(0.0)
            return false
        }

        if (resetPid) {
            prevError = error
            integralError = 0.0
            resetPid = false
        }

        if (error * prevError < 0.0) {
            integralError = 0.0
        }

        if (abs(error) <= I_ZONE_TICKS) {
            integralError += error * dt
            val activeIClamp: Double =
                if (abs(error) <= MIN_POWER_ERROR_TICKS) NEAR_TARGET_I_CLAMP
                else MAX_I
            integralError = when {
                integralError > activeIClamp -> activeIClamp
                integralError < -activeIClamp -> -activeIClamp
                else -> integralError
            }
        } else {
            integralError *= 0.9
        }

        val derivative = if (dt > 0.0) (error - prevError) / dt else 0.0
        prevError = error
        val output: Double = (P * error) + (I * integralError) + (D * derivative)

        Log.i(
            "TurretImpl",
            "P %.2f I %.2f D %.2f => %.2f".format(
                P * error,
                I * integralError,
                D * derivative,
                output
            )
        )
        val output2 = when {
            output > 1.0 -> 1.0
            output < -1.0 -> -1.0
            abs(error) > MIN_POWER_ERROR_TICKS && abs(output) < BASE_POWER -> output.sign * BASE_POWER
            else -> output
        }

        hw.setTurretPower(output2)

        return false
    }

    fun currentPosition() = hw.turretEncoder.getCurrentPosition()
    fun velocity() = hw.turretEncoder.getVelocity()
    fun suspend() {
        suspended = true
    }
    fun resume() {
        suspended = false
    }
    fun setPower(power: Double) {
        hw.setTurretPower(power)
    }
}