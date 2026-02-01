package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import android.util.Size
import com.qualcomm.robotcore.util.ElapsedTime
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.tasks.compose
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class AprilTag(val camera: CameraName) {
    var aprilTagProcessor: AprilTagProcessor? = null; private set
    var visionPortal: VisionPortal? = null; private set
    var obelisk: Motif? = null; private set

    companion object {

        val RESOLUTION = Size(1280, 960)
        @JvmField
        val OBELISK = mapOf(
            21 to Motif.GPP,
            22 to Motif.PGP,
            23 to Motif.PPG
        )
        init {
            systemPackages.add(AprilTag::class.qualifiedName!!)
        }
    }

    fun stop() {
        visionPortal?.stopStreaming()
    }


    fun setupAprilTag(targetExposure: Long, targetGain: Int): Task<*> = object : Task.Anonymous() {

        override fun onStart() {
            aprilTagProcessor = AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build()

            visionPortal = VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(RESOLUTION)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
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

//            val exposure = visionPortal.getCameraControl(ExposureControl::class.java)
//            val gain = visionPortal.getCameraControl(GainControl::class.java)
//
//            exposure?.let {
//                it.mode = ExposureControl.Mode.Manual
//                val minExposure = it.getMinExposure(TimeUnit.MILLISECONDS) + 1L
//                val maxExposure = it.getMaxExposure(TimeUnit.MILLISECONDS)
//                val currentExposure = Math.clamp(targetExposure, minExposure, maxExposure)
//                it.setExposure(currentExposure, TimeUnit.MILLISECONDS)
//            }
//
//            gain?.let {
//                val minGain = it.minGain
//                val maxGain = it.maxGain
//                val currentGain = clamp(targetGain, minGain, maxGain)
//                it.gain = currentGain
//            }
        }
    }

    fun readObelisk(timeout: Double) = object : Task.Anonymous() {
        val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)

        override fun onStart() {
            timer.reset()
        }

        override fun onTick(): Boolean {
            if (timeout > 0 && timer.time() > timeout) return true
            val detections = aprilTagProcessor!!.detections
            for (detection in detections) {
                if (detection.id !in OBELISK) continue
                obelisk = OBELISK[detection.id]!!
                Log.i("AprilTag", "Got detection $obelisk")
                return true
            }
            return false
        }
    }

    fun liveReadObelisk() = compose {
        onTick {
            val detections = aprilTagProcessor!!.detections
            val obelisks = detections.filter { it.id in OBELISK }
            if (obelisks.size == 1) obelisk = OBELISK[obelisks[0].id]!!
            false
        }
    }
}