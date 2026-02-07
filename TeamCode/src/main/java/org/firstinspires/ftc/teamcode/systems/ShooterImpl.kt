package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import kotlin.math.abs

class ShooterImpl(private val hw: CompBot2Hardware) : Task<ShooterImpl>() {
    companion object {
        private val LOCK_ROOT = Lock.StrLock("shooter_impl")

        /**
         * Encoder ticks per second.
         */
        private const val ACCEPTABLE_VELOCITY_DIFF = 30.0

        init {
            systemPackages.add(ShooterImpl::class.qualifiedName!!)
        }
    }

    val lock = LOCK_ROOT.derive()
    private var target = 0.0

    override fun onStart() {
        hw.setupShooterVel()
    }

    override fun onTick(): Boolean {
        hw.copyShooterPower()
        return false
    }

    fun setTarget(vel: Double) {
        target = vel
        hw.setShoot1Vel(vel)
    }

    fun setTargetAsync(vel: Double) = setTargetAsync { vel }
    inline fun setTargetAsync(crossinline vel: () -> Double) = OneShot { setTarget(vel()) }

    @JvmOverloads
    fun awaitTarget(minimumDuration: Double = 0.5, maximumDuration: Double = -1.0) =
        object : Anonymous() {
            init {
                require(lock)
            }

            private val targetDuration = (minimumDuration * 1e9).toLong()
            private var lastMetAt = 0L
            private var start = 0L

            override fun onStart() {
                val now = System.nanoTime()
                lastMetAt = now
                start = now
            }

            override fun onTick(): Boolean {
                val now = System.nanoTime()
                if (maximumDuration > 0 && now - start > maximumDuration * 1e9) return true
                val currentVelocity = hw.shoot1Vel
                if (!(abs(currentVelocity - target) < ACCEPTABLE_VELOCITY_DIFF)) {
                    lastMetAt = now
                    return false
                }
                Log.i("Shooter", "$currentVelocity -> $target = ${abs(currentVelocity - target)}")
                return (now - lastMetAt) >= targetDuration
            }
        }

    @JvmOverloads
    fun setTargetAndWait(
        velocity: Double,
        minDuration: Double = 0.5,
        maxDuration: Double = -1.0
    ) = setTargetAndWait(minDuration, maxDuration) { velocity }

    @JvmOverloads
    inline fun setTargetAndWait(
        minDuration: Double = 0.5,
        maxDuration: Double = -1.0,
        crossinline velocity: () -> Double
    ) = VirtualGroup {
        add(OneShot { setTarget(velocity()) })
            .then(awaitTarget(minDuration, maxDuration))
    }.require(lock)
}