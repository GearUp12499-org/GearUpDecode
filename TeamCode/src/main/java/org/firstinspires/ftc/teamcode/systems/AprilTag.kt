package org.firstinspires.ftc.teamcode.systems

import android.util.Size
import androidx.core.math.MathUtils
import com.qualcomm.robotcore.util.ElapsedTime
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.tools.wrapAngle
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit.MILLISECONDS
import kotlin.math.PI
import kotlin.math.absoluteValue

class AprilTag(val gsc: CameraName) {
    enum class Obelisk {
        GPP,
        PGP,
        PPG,
    }

    companion object {
        @JvmField
        val GSC_POSITION: Position = Position(
            DistanceUnit.INCH, -5.15625, -7.34375, 0.0, 0
        )
        @JvmField
        val GSC_ORIENTATION: YawPitchRollAngles = YawPitchRollAngles(
            AngleUnit.DEGREES, 180.0, -90.0, 0.0, 0
        )
        @JvmField
        val GSC_RESOLUTION: Size = Size(1600, 1200)

        @JvmField
        val OBELISK = mapOf(
            21 to Obelisk.GPP,
            22 to Obelisk.PGP,
            23 to Obelisk.PPG
        )

        @JvmField
        val GOAL_TAGS = setOf(
            20, // blue
            24, // red
        )

        const val BEARING_TOO_BIG = 30 //deg

        init {
            systemPackages.add(AprilTag::class.qualifiedName!!)
        }

    }

    var visionPortal: VisionPortal? = null
    var aprilTagProcessor: AprilTagProcessor? = null
    var minExposure = 0L
    var maxExposure = 0L
    var currentExposure = 0L
    var minGain = 0
    var maxGain = 0
    var currentGain = 0
    var obelisk: Obelisk? = null

    fun setupAprilTag(targetExposure: Long, targetGain: Int): Task<*> = object : Task.Anonymous() {
        override fun onStart() {
            aprilTagProcessor = AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(GSC_POSITION, GSC_ORIENTATION)
                .build()

            visionPortal = VisionPortal.Builder()
                .setCamera(gsc)
                .setCameraResolution(GSC_RESOLUTION)
                .addProcessor(aprilTagProcessor)
                .build()
        }

        override fun onTick(): Boolean {
            if (visionPortal!!.cameraState == VisionPortal.CameraState.STREAMING) {
                setUpControls()
                return true
            }
            return false
        }

        fun setUpControls() {
            val visionPortal = visionPortal!!

            val exposure = visionPortal.getCameraControl(ExposureControl::class.java)
            val gain = visionPortal.getCameraControl(GainControl::class.java)

            exposure?.let {
                it.mode = ExposureControl.Mode.Manual
                minExposure = it.getMinExposure(MILLISECONDS) + 1L
                maxExposure = it.getMaxExposure(MILLISECONDS)
                currentExposure = Math.clamp(targetExposure, minExposure, maxExposure)
                it.setExposure(currentExposure, MILLISECONDS)
            }

            gain?.let {
                minGain = it.minGain
                maxGain = it.maxGain
                currentGain = MathUtils.clamp(targetGain, minGain, maxGain)
                it.gain = currentGain
            }
        }
    }

    fun readObelisk(timeout: Double) = object: Task.Anonymous() {
        val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)

        override fun onStart() {
            timer.reset()
        }

        override fun onTick(): Boolean {
            if (timer.time() > timeout) return true
            val detections = aprilTagProcessor!!.detections
            for (detection in detections) {
                if (detection.id !in OBELISK.keys) continue
                obelisk = OBELISK[detection.id]!!
                return true
            }
            return false
        }
    }

    fun readPosition(): REmover.RobotPose? {
        val detections = aprilTagProcessor?.detections ?: return null
        val detection = detections.filter {
            it.id in GOAL_TAGS && it.ftcPose.bearing.absoluteValue < BEARING_TOO_BIG
        }.minByOrNull {
            it.ftcPose.range
        } ?: return null

        val xyz = detection.robotPose.position
        val pry = detection.robotPose.orientation

        return REmover.RobotPose(
            x = -xyz.x,
            y = -xyz.y,
            a = (pry.getYaw(AngleUnit.RADIANS) - PI / 2).wrapAngle()
        )
    }
}