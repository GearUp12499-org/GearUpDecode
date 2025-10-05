package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.api.BuiltInTags
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper
import org.firstinspires.ftc.teamcode.hardware.HardwareName
import org.firstinspires.ftc.teamcode.perform.PerformanceHelpers
import java.lang.System.nanoTime


@TeleOp
class PinpointProfiler : LinearOpMode() {
    companion object {
        const val MAX_DATA_STORED = 100_000_000 / 4 // 100 MiB of ints
    }

    // lazy just in case FTC SDK is a dumbass and instantiates it multiple times
    val profdata by lazy {
        IntArray(MAX_DATA_STORED) { -1 }
    }

    class Hw(map: HardwareMap) : HardwareMapper(map) {
        @HardwareName("navx")
        lateinit var navX: NavxMicroNavigationSensor

        @HardwareName("PinPoint")
        lateinit var pinpoint: GoBildaPinpointDriver

        @HardwareName("imu")
        lateinit var builtInIMU: IMU

        init {
        }
    }

    override fun runOpMode() {

        val hw = Hw(hardwareMap)
        val sched = FastScheduler()
        val imuOptions: IMU.Parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            )
        )
        val setup = sched.add(object : Task.Anonymous() {
            override fun onStart() {
                hw.builtInIMU.initialize(imuOptions)
                hw.builtInIMU.resetYaw()
                hw.navX.initialize()
                hw.pinpoint.resetPosAndIMU()
            }

            override fun onTick(): Boolean {
                telemetry.addLine("Waiting for calibration to finish.")
//                val navXCalib = hw.navX.isCalibrating
//                telemetry.addData("NavX", if (navXCalib) "..." else "done")
                hw.pinpoint.update()
                val pinpointCalib =
                    hw.pinpoint.deviceStatus == GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                telemetry.addData("Pinpoint", if (pinpointCalib) "..." else "done")
                telemetry.addData("Pinpoint status", hw.pinpoint.deviceStatus)
                telemetry.update()
                return !(pinpointCalib)
            }
        })

        var ptr = 0

        setup.then(object : Task.Anonymous() {

            private val tags = setOf(BuiltInTags.DAEMON)
            override fun getTags(): Set<String> = tags

            override fun onTick(): Boolean {
                val first = nanoTime()
                hw.pinpoint.update()
                val second = nanoTime()
                profdata[ptr] = (second - first).toInt()
                ptr = (ptr + 1) % MAX_DATA_STORED
                telemetry.addData("Write pointer", ptr)
                telemetry.update()
                return false
            }
        })
//        setup.then(object : Task.Anonymous() {
//            override fun onTick(): Boolean {
//                val navX = hw.navX.getAngularOrientation(
//                    AxesReference.INTRINSIC,
//                    AxesOrder.ZYX,
//                    AngleUnit.DEGREES
//                ).firstAngle
//                val pinpoint = hw.pinpoint.getHeading(AngleUnit.DEGREES)
//                val internal = hw.builtInIMU.getRobotOrientation(
//                    AxesReference.INTRINSIC,
//                    AxesOrder.ZYX,
//                    AngleUnit.DEGREES
//                ).firstAngle
//                telemetry.addData("NavXIMU", "%.1fdeg".format(navX))
//                telemetry.addData("Pinpoint", "%.1fdeg".format(pinpoint))
//                telemetry.addData("Builtin", "%.1fdeg".format(internal))
//                telemetry.update()
//                return false
//            }
//        })

        waitForStart()
        while (opModeIsActive()) sched.tick()

        val file = PerformanceHelpers.findSuitableFile()
        var written = 0
        file.parentFile?.mkdirs()
        file.writer(Charsets.UTF_8).use {
            it.write("nanos\n")
            for (value in profdata) {
                if (value == -1) break
                it.write("$value\n")
                written++
            }
        }

        Log.i("Profiler", "wrote profile out to $file: $written data points")
    }

}