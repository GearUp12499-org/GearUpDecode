package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Task
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper
import org.firstinspires.ftc.teamcode.hardware.HardwareName
import org.firstinspires.ftc.teamcode.tasks.DAEMON_TAGS


@TeleOp
class IMUComparatorTest : LinearOpMode() {
    class Hw(map: HardwareMap) : HardwareMapper(map) {
        @HardwareName("navx")
        lateinit var navX: NavxMicroNavigationSensor

        @HardwareName("PinPoint")
        lateinit var pinpoint: GoBildaPinpoint2Driver

        @HardwareName("imu")
        lateinit var builtInIMU: IMU

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
                val navXCalib = hw.navX.isCalibrating
                telemetry.addData("NavX", if (navXCalib) "..." else "done")
                val pinpointCalib =
                    hw.pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.CALIBRATING
                telemetry.addData("Pinpoint", if (pinpointCalib) "..." else "done")
                telemetry.addData("Pinpoint status", hw.pinpoint.deviceStatus)
                telemetry.update()
                return !(navXCalib || pinpointCalib)
            }
        })
        setup.then(object : Task.Anonymous() {
            override fun getTags(): Set<String> = DAEMON_TAGS

            override fun onTick(): Boolean {
                hw.pinpoint.update()
                return false
            }
        })
        setup.then(object : Task.Anonymous() {
            override fun onTick(): Boolean {
                val navX = hw.navX.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES
                ).firstAngle
                val pinpoint = hw.pinpoint.getHeading(AngleUnit.DEGREES)
                val internal = hw.builtInIMU.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES
                ).firstAngle
                telemetry.addData("NavXIMU", "%.1fdeg".format(navX))
                telemetry.addData("Pinpoint", "%.1fdeg".format(pinpoint))
                telemetry.addData("Builtin", "%.1fdeg".format(internal))
                telemetry.update()
                return false
            }
        })

        waitForStart()
        while (opModeIsActive()) sched.tick()
    }

}