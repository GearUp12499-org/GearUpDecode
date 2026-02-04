package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.systemPackages
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

class TurretTrack(
    private val ll: Limelight3A,
    private val turret: DcMotorEx,
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
    val targetPose = poseSet.goalAT

    fun track() = TrackTask()

    inner class TrackTask : Anonymous() {
        private var prevError = 0.0
        private var integralError = 0.0
        private var lastT = 0L
        private var lastTx = 0.0
        private var lastIMUError = 0.0
        private var lastIMUEncoderPosAtCapture = 0
        private var lastEncoderPosAtCapture = 0
        private var isDestinationReachable = true
        private var isLimelightTracking = false
        private var resetPID = false

        var distance: Double? = null
            private set

        override fun onStart() {
            ll.start()
            lastT = System.nanoTime()

            turret.power = 0.0
            turret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        private fun computePower(error: Double, deltaT: Double): Double {
            if (abs(error) < DEADBAND) {
                prevError = error
                integralError = 0.0
                return 0.0
            }

            if (resetPID) {
                prevError = error
                integralError = 0.0
                resetPID = false
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
            val now = System.nanoTime()
            val dt = (now - lastT) / 1e9
            lastT = now

            val currentEncoder = turret.currentPosition
            val rawIMUError = getPinpointGoalYawDiff(currentEncoder)
            val refinedIMUError: Double
            if (abs(turret.velocity) < VELOCITY_THRESHOLD) {
                lastIMUError = rawIMUError
                lastIMUEncoderPosAtCapture = currentEncoder
                refinedIMUError = rawIMUError
            } else {
                val deltaTicks = currentEncoder - lastIMUEncoderPosAtCapture
                refinedIMUError = lastIMUError - (deltaTicks / TICKS_PER_DEG)
            }

            val result = ll.latestResult
            var refinedLLError = NaN
            var llVisible = false
            if (result != null && result.isValid) {
                val tags = result.fiducialResults
                val target = tags.firstOrNull { it.fiducialId == targetTag }
                if (target != null) {
                    distance = taToDistance(target.targetArea)
                    llVisible = true
                    // TODO: REUSE turret.velocity
                    if (abs(turret.velocity) < VELOCITY_THRESHOLD) {
                        lastTx = target.targetXDegrees
                        lastEncoderPosAtCapture = currentEncoder
                        refinedLLError = lastTx
                    } else {
                        val deltaTicks = currentEncoder - lastEncoderPosAtCapture
                        refinedLLError = lastTx - (deltaTicks / TICKS_PER_DEG)
                    }
                }
            }
            Log.i(
                "TurretTrack", when {
                result == null -> "result is null"
                !result.isValid -> "result is invalid"
                else -> {
                    val tags = result.fiducialResults
                    val target = tags.firstOrNull { it.fiducialId == targetTag }
                    when {
                        tags.isEmpty() -> "no results"
                        target == null -> "no matching result"
                        else -> "id ${target.fiducialId}"
                    }
                }
            })

            if (llVisible != isLimelightTracking) {
                Log.i("TurretTrack", if (llVisible) "LOCKED IN" else "Locked out :(")
                isLimelightTracking = llVisible
                resetPID = true
            }

            val finalError = if (llVisible) refinedLLError else refinedIMUError

            val power1 = computePower(finalError, dt)
            val power2 = limit(power1, currentEncoder)
            Log.i(
                "TurretTrack", "mode %s err %.2f pow %.3f %s".format(
                    if (isLimelightTracking) "Limelight" else "IMU",
                    finalError,
                    power2,
                    if (isDestinationReachable) "reachable" else "reachablen't"
                )
            )
            Log.i(
                "TurretTrack", "Limelight meta: pipe %d timestamp %.4f".format(
                    ll.latestResult.pipelineIndex,
                    ll.latestResult.timestamp,
                )
            )

            turret.power = power2

            // TODO: Log
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