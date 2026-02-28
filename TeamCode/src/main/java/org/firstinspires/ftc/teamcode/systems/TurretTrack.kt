package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.limelightvision.LLResult
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.teamcode.PoseSet
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.TURRET_CCW_STOP
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.TURRET_CW_STOP
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import kotlin.math.atan2
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.pow
import kotlin.math.PI
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
        const val TICKS_PER_DEG = CompBot2Hardware.TICKS_PER_DEG
    }

    val targetTag = if (red) TAG_RED else TAG_BLUE
    val targetPose = poseSet.goalAT
    val pipe = if (red) 2 else 7

    private var lastTimestamp: Double = 0.0
    private var lastPoll = markNow()
    var fault = false; private set

    fun track() = TrackTask()
    fun trackLegacy(): ITask<*> = throw IllegalStateException("don't do it") // LegacyTrackTask()

    inner class TrackTask : Anonymous() {
        private var lastT = 0L
        private var isDestinationReachable = true

        var distance: Double? = null
            private set

        override fun onStart() {
            ll.start()
            lastT = System.nanoTime()
        }

        private fun getPoseRobotFromLL(xLL: Double, yLL: Double, thetaTurret: Double, thetaRobot: Double): Pair<Double, Double> {
            // thetaRobot MUST be in radians

            val rTurret = 6.5
            val tOffset = 0.5

            val d = sqrt(
                (rTurret).pow(2.0) +
                (tOffset).pow(2.0) -
                2 * (rTurret) * (tOffset) * cos(PI - thetaTurret)
            )

            val x = asin(
                sin(PI - thetaTurret) * rTurret / d
            )

            val f = d * cos(x)
            val s = d * sin(x)

            val xOff = f * cos(thetaRobot) - s * sin(thetaRobot)
            val yOff = f * sin(thetaRobot) + s * cos(thetaRobot)

            val xRobot = xLL + xOff
            val yRobot = yLL + yOff

            return Pair(xRobot,yRobot)
        }

        private fun getLimelightPose2D(result: LLResult): REmover.RobotPose {
            // Do NOT call if result is not sanitized for being null; Risk of NullObjectReference
            val robotPose: Pose3D = result.botpose
            val limelightX = robotPose.position.x * 39.37 * -1
            val limelightY = robotPose.position.y * 39.37 * -1
            val limelightTheta = robotPose.orientation.yaw
            return REmover.RobotPose(limelightX, limelightY, limelightTheta)
        }

        private fun taToDistance(ta: Double): Double {
            return sqrt(56.0 / ta) - 5.82
        }

        private fun getPinpointGoalYawDiff(currentTurretEncoder: Int): Double {
            // TODO: Possibly change this to getPinpointGoalYaw. Return a raw angle
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

            return -llConventionError // TODO: Why is this backwards
        }

        override fun onTick(): Boolean {
            Log.w(TrackTask::class.simpleName, "is running")
            // Determine whether to use pinpoint or limelight
            val result = ll.latestResult
            var useLL = false

            // TODO: If turret moving too fast, keep useLL false
            if (result != null && result.isValid) {
                useLL = true
            }

//            useLL = false
            if (useLL) {
                val actualPipeline = result.pipelineIndex
                if (actualPipeline != pipe) {
                    Log.w(
                        TrackTask::class.simpleName,
                        "Wrong pipeline ($actualPipeline), trying to switch to $pipe"
                    )
                    ll.pipelineSwitch(pipe)
                    return false
                }
                val llPose = getLimelightPose2D(result)
                val pinpointPose = pinpoint.position.remover
                Log.w(
                    "Limelight Camera",
                    "(%.2f, %.2f)".format(llPose.x, llPose.y)
                )
                val (ll2RobotX, ll2RobotY) = getPoseRobotFromLL(
                    llPose.x,
                    llPose.y,
                    (-turret.currentPosition() / TICKS_PER_DEG) * (PI / 180),
                    pinpointPose.a
                )
                Log.w(
                    "Thetas",
                    "[turret = %.2f, robot = %.2f]".format((-turret.currentPosition() / TICKS_PER_DEG) * (PI / 180), pinpointPose.a)
                )
                Log.w(
                    "Limelight Robot",
                    "(%.2f, %.2f)".format(ll2RobotX, ll2RobotY)
                )
                Log.w(
                    "Pinpoint Pose",
                    "(%.2f, %.2f)".format(pinpointPose.x,  pinpointPose.y)
                )
                val tags = result.fiducialResults
                val target = tags.firstOrNull { it.fiducialId == targetTag }
                if (target == null) {
                    return false
                }

                Log.w(
                    "Tracking Mode",
                    ">>> LIMELIGHT MODE"
                )

                // TODO: Use pinpoint distance
                distance = taToDistance(target.targetArea)
                Log.w(
                    "Limelight Predicted Distance",
                    "%.4f".format(distance)
                )
                Log.w(
                    "Limelight Baring",
                    "%.4f".format(target.targetXDegrees)
                )
                turret.setDeltaTarget(-target.targetXDegrees)
                return false
            }

            val currentEncoder = turret.currentPosition()
            Log.w(
                "Tracking Mode",
                ">>> PINPOINT MODE"
            )
            Log.w(
                "Pinpoint Yaw Diff",
                "%.4f".format(getPinpointGoalYawDiff(currentEncoder))
            )
            turret.setDeltaTarget(getPinpointGoalYawDiff(currentEncoder))
            return false

        }

        override fun onFinish(completedNormally: Boolean) {
            ll.stop()
        }
    }

    override fun onTick(): Boolean {
        Log.w(TurretTrack::class.simpleName, "is running")
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