package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.PoseSet
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.TURRET_CCW_STOP
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.TURRET_CW_STOP
import java.lang.Math.clamp
import kotlin.Double.Companion.NaN
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.sqrt
import kotlin.time.Duration.Companion.seconds
import kotlin.time.TimeSource.Monotonic.markNow

class TurretTrack(
    private val ll: Limelight3A,
    private val turret: TurretImpl,
    private val pinpoint: GoBildaPinpoint2Driver,
    poseSet: PoseSet,
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
        const val TICKS_PER_DEG = CompBot2Hardware.TICKS_PER_DEG
        const val DEADBAND = 1.0
        const val MAX_I = 20_000.0
        const val KP = 0.02
        const val KI = 0.000_07
        const val KD = 0.005
        const val MAX_POWER = .6
        const val SOFT_LIMIT_BUFFER = 200
    }

    val targetTag = if (red) TAG_RED else TAG_BLUE
    val targetPose = poseSet.goalAT
    val pipe = if (red) 2 else 7

    private var lastTimestamp: Double = 0.0
    private var lastPoll = markNow()
    var fault = false; private set


    fun track() = TrackTask()
    fun trackLegacy() = LegacyTrackTask()

    inner class TrackTask : Anonymous() {
        private var lastT = 0L
        private var isDestinationReachable = true

        var distance: Double? = null
            private set

        override fun onStart() {
            ll.start()
            lastT = System.nanoTime()
            turret.suspend()
        }

        private fun taToDistance(ta: Double): Double {
            return sqrt(56.0 / ta) - 5.82
        }

        private fun getPinpointGoalYawDiff(currentTurretEncoder: Int): Double {
            val currentPose = pinpoint.position.remover

            val x = targetPose.x - currentPose.x
            val y = targetPose.y - currentPose.y
            val goalAngle = atan2(y, x)
            val goalAngleDeg = goalAngle.wrapAngle().toDeg()
            val botHeading = currentPose.a.toDeg()
            val turretRotation = currentTurretEncoder / TICKS_PER_DEG
            val turretWorldHeading = (botHeading + 180.0 - turretRotation).wrapAngleDeg()

            // TODO: log

            val error = (goalAngleDeg - turretWorldHeading).wrapAngleDeg()
            val llConventionError = -error
            val targetTicks = currentTurretEncoder + (llConventionError * TICKS_PER_DEG).toInt()
            isDestinationReachable = targetTicks in TURRET_CCW_STOP..TURRET_CW_STOP

            return llConventionError
        }

        override fun onTick(): Boolean {
            // Determine whether to use pinpoint or limelight
            val result = ll.latestResult
            var useLL = false

            // TODO: If turret moving too fast, keep useLL false
            if (result != null && result.isValid) {
                useLL = true
            }

            if (useLL) {
                val actualPipeline = result.pipelineIndex
                if (actualPipeline != pipe) {
                    Log.w(
                        "TurretTrack",
                        "Wrong pipeline ($actualPipeline), trying to switch to $pipe"
                    )
                    ll.pipelineSwitch(pipe)
                    return false
                }
                val tags = result.fiducialResults
                val target = tags.firstOrNull { it.fiducialId == targetTag }
                if (target == null) {
                    return false
                }

                // TODO: Use pinpoint distance
                distance = taToDistance(target.targetArea)
                turret.setDeltaTarget(target.targetXDegrees)
                return false
            }

            val currentEncoder = turret.currentPosition()
            turret.setDeltaTarget(getPinpointGoalYawDiff(currentEncoder))
            return false

        }

        override fun onFinish(completedNormally: Boolean) {
            ll.stop()
            turret.resume()
        }
    }

    inner class LegacyTrackTask : Anonymous() {
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

            turret.suspend()
        }

        private fun noResult() {
            turret.setPower(0.0)
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
            if (pos >= TURRET_CW_STOP - SOFT_LIMIT_BUFFER && power > 0) {
                val distanceToLimit = TURRET_CW_STOP - pos
                val scaleFactor = distanceToLimit / SOFT_LIMIT_BUFFER.toDouble()
                result *= max(0.0, scaleFactor)
            }

            if (pos <= TURRET_CCW_STOP + SOFT_LIMIT_BUFFER && power < 0) {
                val distanceToLimit = pos - TURRET_CCW_STOP
                val scaleFactor = distanceToLimit / SOFT_LIMIT_BUFFER.toDouble()
                result *= max(0.0, scaleFactor)
            }

            if (pos >= TURRET_CW_STOP && power > 0) {
                return 0.0
            }
            if (pos <= TURRET_CCW_STOP && power < 0) {
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

            val currentEncoder = turret.currentPosition()

            val tx: Double
            if (abs(turret.velocity()) < VELOCITY_THRESHOLD) {
                lastTx = target.targetXDegrees
                lastEncoderPosAtCapture = currentEncoder
                tx = lastTx
            } else {
                val deltaTicks = currentEncoder - lastEncoderPosAtCapture
                tx = lastTx - (deltaTicks / TICKS_PER_DEG)
            }

            val power1 = computePower(tx, dt)
            val power2 = limit(power1, currentEncoder)

            turret.setPower(power2)
            return false
        }

        override fun onFinish(completedNormally: Boolean) {
            ll.stop()
            turret.resume()
        }
    }

    override fun onTick(): Boolean {
        assert(false)
        val timestamp = ll.latestResult.timestamp
        val nowTs = markNow()
        if (timestamp != lastTimestamp) {
            lastTimestamp = timestamp
            lastPoll = nowTs
        }
        if (nowTs - lastPoll > 1.seconds) {
            // IoC
            Log.w("TurretTrack", "LL may be compromised: last stamp $timestamp, ${nowTs - lastPoll} ago")
            fault = true
        } else {
            fault = false
        }

        return false
    }

    override fun onStart() {
        ll.stop()
        ll.setPollRateHz(150)
        ll.pipelineSwitch(pipe)
    }

    override fun getTags() = TAGS
}