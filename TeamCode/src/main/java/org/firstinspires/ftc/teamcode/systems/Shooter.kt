package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark.systemPackages
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.tasks.DAEMON_TAGS
import kotlin.math.abs

class Shooter(private val motor: DcMotorEx, private val angle: Servo) : Task<Shooter>() {
    companion object {
        private val LOCK_ROOT = Lock.StrLock("shooter_impl")

        /**
         * Encoder ticks per second.
         */
        private const val ACCEPTABLE_VELOCITY_DIFF = 50.0

        init {
            systemPackages.add(Shooter::class.qualifiedName!!)
        }
    }

    var targetVelocity: Double = 0.0
    private var currentVelocity: Double = 0.0

    init {
        require(CompBotHardware.Locks.SHOOTER)
    }

    val lock = LOCK_ROOT.derive()

    override fun onStart() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.mode = RunMode.RUN_USING_ENCODER
        motor.velocity = 0.0
    }

    override fun onTick(): Boolean {
        currentVelocity = motor.velocity
        return false
    }

    override fun onFinish(completedNormally: Boolean) {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = RunMode.RUN_WITHOUT_ENCODER
        motor.power = 0.0
    }

    override fun getTags(): Set<String> = DAEMON_TAGS

    fun isAtTarget() = abs(currentVelocity - targetVelocity) < ACCEPTABLE_VELOCITY_DIFF
    fun setTarget(velo: Double) {
        targetVelocity = velo
        motor.velocity = velo
    }

    /**
     * Wait to reach the target velocity. Does not account for overshooting.
     */
    fun waitForTargetSimple() = WaitUntil(::isAtTarget)

    /**
     * Wait to reach the target velocity for [minimumDuration] seconds continuously.
     *
     * Setting the [minimumDuration] to `0.0` is effectively the same as [waitForTargetSimple].
     */
    @JvmOverloads
    fun waitForTargetHold(minimumDuration: Double = 0.5) = object : Anonymous() {
        init {
            require(lock)
        }

        private val targetDuration = (minimumDuration * 1e9).toLong()
        private var lastMetAt = 0L

        override fun onTick(): Boolean {
            val now = System.nanoTime()
            Log.i("Shooter", "%.2f -> %.2f => %.2f".format(currentVelocity, targetVelocity, abs(currentVelocity - targetVelocity)))
            if (!isAtTarget()) {
                lastMetAt = now
                return false
            }
            return (now - lastMetAt) >= targetDuration
        }

    }

    /**
     * Set the target velocity, then wait to stabilize on that velocity.
     *
     * Set the [minDuration] to `0.0` to complete immediately when the target velocity is met,
     * similar to [waitForTargetSimple].
     */
    @JvmOverloads
    fun setTargetAndWait(velocity: Double, minDuration: Double = 0.5) = VirtualGroup {
        add(OneShot { setTarget(velocity) })
            .then(waitForTargetHold(minDuration))
    }.require(lock)

    /**
     * Set the target velocity to 0 (as a task.)
     */
    fun stopSoft() = OneShot {
        setTarget(0.0)
    }.require(lock)
}