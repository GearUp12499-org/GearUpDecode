package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import io.github.gearup12499.taskshark.Lock
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import kotlin.math.abs

class Shooter(private val motor: DcMotorEx, private val angle: Servo) : Task<Shooter>() {
    companion object {
        private val TAGS = setOf(BuiltInTags.DAEMON)
        private val LOCK_ROOT = Lock.StrLock("shooter_impl")

        /**
         * Encoder ticks per second.
         */
        private const val ACCEPTABLE_VELOCITY_DIFF = 50.0
    }

    var targetVelocity: Double = 0.0
    private var currentVelocity: Double = 0.0

    init {
        require(CompBotHardware.Locks.INDEXER)
    }

    val lock = LOCK_ROOT.derive()

    override fun onStart() {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.mode = RunMode.RUN_USING_ENCODER
        motor.velocity = 0.0
    }

    override fun onTick(): Boolean {
        motor.velocity = targetVelocity
        currentVelocity = motor.velocity
        return false
    }

    override fun onFinish(completedNormally: Boolean) {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.power = 0.0
    }

    override fun getTags(): Set<String> = TAGS

    fun isAtTarget() = abs(currentVelocity - targetVelocity) < ACCEPTABLE_VELOCITY_DIFF

    /**
     * Wait to reach the target velocity. Does not account for overshooting.
     */
    fun waitForTargetSimple() = WaitUntil(::isAtTarget)

    /**
     * Wait to reach the target velocity for [minimumDuration] seconds continuously.
     */
    @JvmOverloads
    fun waitForTargetHold(minimumDuration: Double = 0.5) = object : Anonymous() {
        private val targetDuration = (minimumDuration * 1e9).toLong()
        private var lastMetAt = 0L

        override fun onTick(): Boolean {
            val now = System.nanoTime()
            if (!isAtTarget()) {
                lastMetAt = now
                return false
            }
            return (now - lastMetAt) >= targetDuration
        }

    }
}