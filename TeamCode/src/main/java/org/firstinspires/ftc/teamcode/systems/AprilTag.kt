package org.firstinspires.ftc.teamcode.systems

import android.util.Size
import androidx.core.math.MathUtils
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit.MILLISECONDS

class AprilTag(val gsc: CameraName) {
    companion object {
        val GSC_POSITION: Position = Position(
            DistanceUnit.INCH, 0.0, 8.314, 7.73, 0
        )
        val GSC_ORIENTATION: YawPitchRollAngles = YawPitchRollAngles(
            AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0
        )
        val GSC_RESOLUTION: Size = Size(1600, 1200)

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

            val exposure = visionPortal.getCameraControl(ExposureControl::class.java);
            val gain = visionPortal.getCameraControl(GainControl::class.java);

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
}