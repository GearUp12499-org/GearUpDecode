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
import kotlin.math.hypot
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
    fun trackLegacy() = LegacyTrackTask() // : ITask<*> = throw IllegalStateException("don't do it")

    inner class TrackTask : Anonymous() {
        private var lastT = 0L
        private var lastPinpointUpdate = lastT
        private var pinpointErrorX = 0.0
        private var pinpointErrorY = 0.0
        private var pinpointValX = 0.0
        private var pinpointValY = 0.0
        private var limelightX = 0.0
        private var limelightY = 0.0
        private var llErr = 0.0
        private var isDestinationReachable = true
        private var TURRET_POWER_THRESHOLD = 0.8

        var distance: Double? = null
            private set

        override fun onStart() {
            ll.start()
            lastT = System.nanoTime()
        }

        fun resetPinpointErrorXY() {
            pinpointErrorX = 0.0
            pinpointErrorY = 0.0
        }

        fun getPinpointXY(): Pair<Double, Double> {
            return Pair(pinpointValX, pinpointValY)
        }

        fun getLimelightXY(): Pair<Double, Double> {
            return Pair(limelightX, limelightY)
        }

        fun getPinpointErrors(): Triple<Double, Double, Double> {
            return Triple(pinpointErrorX, pinpointErrorY, llErr)
        }

        private fun getPoseRobotFromLL(
            xLL: Double,
            yLL: Double,
            thetaTurret: Double,
            thetaRobot: Double
        ): Pair<Double, Double> {
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

            return Pair(xRobot, yRobot)
        }

        private fun llErrDynamic(xDiff: Double, yDiff: Double): Double {
            if (-0.04*xDiff + 0.047*yDiff + 0.06 < 1.0) {
                return 1.0
            }
            return -0.04*xDiff + 0.047*yDiff + 0.06
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
            return sqrt(56.0 / ta) - 5.82 //use for legacy task
        }

        private fun getTargetFromPinpoint(ppErrX: Double, ppErrY: Double, currentPose: REmover.RobotPose): Double {
            val x = targetPose.x - (currentPose.x + ppErrX)
            val y = targetPose.y - (currentPose.y + ppErrY)
            val robotAngle = currentPose.a.toDeg()
            val goalAngle = atan2(y, x).toDeg()

            var targetAngle = (180 + goalAngle - robotAngle)

            while (targetAngle >= 180.0) targetAngle -= 360.0
            while (targetAngle < -180.0) targetAngle += 360.0

            return(targetAngle)
        }

        override fun onTick(): Boolean {
            // i don't know why stop() isn't stopping it
            if (getState() != ITask.State.Ticking) return false
            // Determine whether to use pinpoint or limelight
            val result = ll.latestResult
            var useLL = false

            // TODO: If turret moving too fast, keep useLL false
            if (result != null && result.isValid) {
                useLL = true
            }
            val pinpointPose = pinpoint.position.remover
            pinpointValX = pinpointPose.x
            pinpointValY = pinpointPose.y

            // Set const threshold for power
            if (useLL && turret.getPower() < TURRET_POWER_THRESHOLD) {
                lastT = System.nanoTime()
                val actualPipeline = result.pipelineIndex
                if (actualPipeline != pipe) {
                    Log.w(
                        this::class.simpleName,
                        "Wrong pipeline ($actualPipeline), trying to switch to $pipe"
                    )
                    ll.pipelineSwitch(pipe)
                    return false
                }
                val llPose = getLimelightPose2D(result)
                val (ll2RobotX, ll2RobotY) = getPoseRobotFromLL(
                    llPose.x,
                    llPose.y,
                    (-turret.currentPosition() / TICKS_PER_DEG) * (PI / 180),
                    pinpointPose.a
                )
                limelightX = ll2RobotX
                limelightY = ll2RobotY
                val pinpointX = pinpointPose.x + pinpointErrorX
                val pinpointY = pinpointPose.y + pinpointErrorY
                val dx = ll2RobotX - pinpointX
                val dy = ll2RobotY - pinpointY
                val distanceLL2pp = hypot(dx.pow(2.0), dy.pow(2.0))
                llErr = llErrDynamic(pinpointPose.x - targetPose.x, pinpointPose.y - targetPose.y) //2.44 // avg error from data collect on 2/28
                if (distanceLL2pp > llErr && lastT - lastPinpointUpdate > 1e9) {
                    lastPinpointUpdate = lastT
                    val alpha = (0.5 * (llErr + distanceLL2pp)) / distanceLL2pp
                    val guessPointX = (alpha * pinpointX) + ((1 - alpha) * ll2RobotX)
                    val guessPointY = (alpha * pinpointY) + ((1 - alpha) * ll2RobotY)
                    pinpointErrorX += (guessPointX - pinpointX)
                    pinpointErrorY += (guessPointY - pinpointY)
                }

                Log.i(
                    this::class.simpleName,
                    "TrackTask: cameraXY(%.2f %.2f) theta(t=%.2f r=%.2f)\nll(%.2f %.2f) pp(%.2f %.2f) err(%.2f %.2f)".format(
                        llPose.x, llPose.y,
                        (-turret.currentPosition() / TICKS_PER_DEG) * (PI / 180), pinpointPose.a,
                        ll2RobotX, ll2RobotY,
                        pinpointX, pinpointY,
                        pinpointErrorX, pinpointErrorY
                    )
                )
                val tags = result.fiducialResults
                val target = tags.firstOrNull { it.fiducialId == targetTag }
                if (target == null) {
                    return false
                }

                // TODO: Use pinpoint distance
                val deltaX = targetPose.x - pinpointX
                val deltaY = targetPose.y - pinpointY
                val shootOffset = 0 // corrected center of robot
                distance = hypot(deltaX, deltaY) - shootOffset

                turret.setDeltaTarget(-target.targetXDegrees)

                Log.i(
                    TrackTask::class.simpleName,
                    "Limelight mode info: dist %.4f bearing %.4f".format(
                        distance,
                        target.targetXDegrees
                    )
                )

                return false
            }

            val currentEncoder = turret.currentPosition()

            val yaw = getTargetFromPinpoint(
                pinpointErrorX,
                pinpointErrorY,
                pinpointPose
            )

            turret.setTarget(yaw)
            val pinpointX = pinpointPose.x + pinpointErrorX
            val pinpointY = pinpointPose.y + pinpointErrorY
            val dx = targetPose.x - pinpointX
            val dy = targetPose.y - pinpointY
            val shootOffset = 0 // corrected center of robot
            distance = hypot(dx, dy) - shootOffset
            Log.i(
                this::class.simpleName,
                "Pinpoint mode info: yaw %.4f dist %.4f".format(
                    yaw,
                    distance
                )
            )
            return false
        }

        override fun onFinish(completedNormally: Boolean) {
            ll.pause()
            turret.setTarget(0.0)
        }
    }

    inner class LegacyTrackTask : Anonymous() {
        private var lastT = 0L

        var distance: Double? = null
            private set

        override fun onStart() {
            ll.start()
            lastT = System.nanoTime()
        }

        private fun taToDistance(ta: Double): Double {
            return sqrt(56.0 / ta) - 5.82 // use for legacy task
        }

        override fun onTick(): Boolean {
            if (getState() != ITask.State.Ticking) return false
            val result = ll.latestResult

            if (result == null || !result.isValid) {
                Log.w(this::class.simpleName, "Legacy: invalid/null result")
                return false
            }

            val actualPipeline = result.pipelineIndex
            if (actualPipeline != pipe) {
                Log.w(
                    this::class.simpleName,
                    "Wrong pipeline ($actualPipeline), trying to switch to $pipe"
                )
                ll.pipelineSwitch(pipe)
                return false
            }
            val tags = result.fiducialResults
            val target = tags.firstOrNull { it.fiducialId == targetTag }

            if (target == null) {
                Log.w(this::class.simpleName, "Legacy: no matching results ($tags)")
                return false
            }

            distance = taToDistance(target.targetArea)

            turret.setDeltaTarget(-target.targetXDegrees)
            Log.i(
                this::class.simpleName,
                "Legacy: Limelight info: dist %.4f bearing %.4f".format(
                    distance,
                    target.targetXDegrees
                )
            )
            return false
        }

        override fun onFinish(completedNormally: Boolean) {
            ll.pause()
            turret.setTarget(0.0)
        }
    }

    override fun onTick(): Boolean {
        val timestamp = ll.latestResult.timestamp
        val nowTs = markNow()
        if (timestamp != lastTimestamp) {
            lastTimestamp = timestamp
            lastPoll = nowTs
        }
        if (nowTs - lastPoll > 1.seconds) {
            // IoC
            Log.w(
                "TurretTrack",
                "LL may be compromised: last stamp $timestamp, ${nowTs - lastPoll} ago"
            )
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