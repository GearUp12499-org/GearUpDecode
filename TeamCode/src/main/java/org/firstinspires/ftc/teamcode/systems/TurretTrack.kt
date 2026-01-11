package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import java.lang.Math.clamp
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

class TurretTrack(
    private val ll: Limelight3A,
    private val turret: DcMotorEx,
    private val red: Boolean
) : Task<TurretTrack>() {
    companion object {
        init {
            systemPackages.add(TurretTrack::class.qualifiedName!!)
        }

        private val TAGS = setOf(BuiltInTags.DAEMON)

        const val TAG_RED = 24
        const val TAG_BLUE = 20
        const val VELOCITY_THRESHOLD = 50
        const val TICKS_PER_DEG = CompBot2Hardware.TURRET_CW_90 / 90.0
        const val DEADBAND = 1.0
        const val MAX_I = 0.2
        const val KP = 0.06
        const val KI = 0.0003
        const val KD = 0.0005
        const val MAX_POWER = 0.8
        const val SOFT_LIMIT_BUFFER = 20
    }

    val targetTag = if (red) TAG_RED else TAG_BLUE

    fun track() = TrackTask()

    inner class TrackTask : Anonymous() {
        private var prevError = 0.0
        private var integralError = 0.0
        private var lastT = 0L
        private var lastTx = 0.0
        private var lastEncoderPosAtCapture = 0

        var distance: Double? = null
            private set

        override fun onStart() {
            ll.start()
            lastT = System.nanoTime()

            turret.power = 0.0
            turret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        private fun noResult() {
            turret.power = 0.0
            prevError = 0.0
        }

        private fun computePower(error: Double, deltaT: Double): Double {
            if (abs(error) < DEADBAND) {
                prevError = error
                integralError = 0.0
                return 0.0
            }

            integralError += error * deltaT
            integralError = clamp(integralError, -MAX_I, MAX_I)

            val p = KP * error
            val i = KI * integralError
            val d = if (deltaT > 0) KD * ((error - prevError) / deltaT) else 0.0

            val out = p + i + d
            return clamp(out, -MAX_POWER, MAX_POWER)
        }

        private fun limit(power: Double, pos: Int): Double {
            var result = power
            if (pos >= CompBot2Hardware.TURRET_CW_STOP - SOFT_LIMIT_BUFFER && power > 0) {
                val distanceToLimit = CompBot2Hardware.TURRET_CW_STOP - pos
                val scaleFactor = distanceToLimit / SOFT_LIMIT_BUFFER.toDouble()
                result *= max(0.0, scaleFactor)
            }

            if (pos <= CompBot2Hardware.TURRET_CCW_STOP + SOFT_LIMIT_BUFFER && power < 0) {
                val distanceToLimit = pos - CompBot2Hardware.TURRET_CCW_STOP
                val scaleFactor = distanceToLimit / SOFT_LIMIT_BUFFER.toDouble()
                result *= max(0.0, scaleFactor)
            }

            if (pos >= CompBot2Hardware.TURRET_CW_STOP && power > 0) {
                return 0.0
            }
            if (pos <= CompBot2Hardware.TURRET_CCW_STOP && power < 0) {
                return 0.0
            }

            return result
        }

        private fun taToDistance(ta: Double): Double {
            return sqrt(56.0 / ta) - 5.82
        }

        override fun onTick(): Boolean {
            val result = ll.latestResult
            if (result == null || !result.isValid) {
                noResult()
                return false
            }

            val tags = result.fiducialResults
            if (tags.isEmpty()) {
                noResult()
                return false
            }

            val target = tags.firstOrNull {
                it.fiducialId == targetTag
            }
            if (target == null) {
                noResult()
                return false
            }

            val now = System.nanoTime()
            val dt = (now - lastT) / 1e9
            lastT = now
            val ta = target.targetArea
            distance = taToDistance(ta)

            val currentEncoder = turret.currentPosition

            val tx: Double
            if (abs(turret.velocity) < VELOCITY_THRESHOLD) {
                lastTx = target.targetXDegrees
                lastEncoderPosAtCapture = currentEncoder
                tx = lastTx
            } else {
                val deltaTicks = currentEncoder - lastEncoderPosAtCapture
                tx = lastTx - (deltaTicks / TICKS_PER_DEG)
            }

            val power1 = computePower(tx, dt)
            val power2 = limit(power1, currentEncoder)

            turret.power = power2
            return false
        }

        override fun onFinish(completedNormally: Boolean) {
            ll.stop()
            turret.power = 0.0
            turret.targetPosition = 0
            turret.mode = DcMotor.RunMode.RUN_TO_POSITION
            turret.power = 1.0
        }
    }

    override fun onTick(): Boolean {
        return false
    }

    override fun onStart() {
        ll.stop()
        ll.setPollRateHz(150)
        if (red) ll.pipelineSwitch(2)
        else ll.pipelineSwitch(7)
    }

    override fun getTags() = TAGS
}