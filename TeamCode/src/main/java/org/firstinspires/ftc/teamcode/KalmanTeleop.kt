package org.firstinspires.ftc.teamcode

import android.support.v4.app.INotificationSideChannel
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.Locks
import org.firstinspires.ftc.teamcode.systems.Kalman
import org.firstinspires.ftc.teamcode.tasks.stopUsing
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

@TeleOp
class KalmanTeleop: LinearOpMode() {
    private lateinit var hw: CompBot2Hardware

    private lateinit var kalman: Kalman

    private lateinit var scheduler: FastScheduler


    override fun runOpMode() {
        TaskSharkAndroid.setup()
        hw = CompBot2Hardware(hardwareMap)

        scheduler = FastScheduler()

        kalman = scheduler.add(Kalman(hw, true, 0.0, 0.0, 0.0))

        hw.pinpoint.resetPosAndIMU()
        hw.pinpoint.setPosition(Pose2D (DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0))

        scheduler.add(DriveTask())
        scheduler.add(kalman)
        waitForStart()

        while (opModeIsActive()) {
            scheduler.tick()

            hw.pinpoint.update()

            var pinpointPose = hw.pinpoint.position

            telemetry.addData("estimated X", kalman.stateX)
            telemetry.addData("estimated Y", kalman.stateY)
            telemetry.addData("estimated Theta", kalman.stateTheta)
            telemetry.addData("pinpoint X", pinpointPose.getX(DistanceUnit.INCH))
            telemetry.addData("pinpoint Y", pinpointPose.getY(DistanceUnit.INCH))
            telemetry.addData("pinpoint Theta", pinpointPose.getHeading(AngleUnit.RADIANS))
            telemetry.update()
        }





    }

    private inner class DriveTask : Task<DriveTask>() {
        private inline val sch: Scheduler get() = super.scheduler!!

        override fun onTick(): Boolean {
            val sch = sch

            mecanumDispatcher(sch)

            return false

        }

        fun mecanumDispatcher(sch: Scheduler) {
            val y = -gamepad1.left_stick_y.toDouble()
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()
            if (sch.getLockOwner(Locks.DRIVE_MOTORS) == null) mecanum(y, x, rx)
            val pushValue = max(hypot(x, y), rx)
            if (pushValue >= Options.DRIVE_PUSH_TO_OVERRIDE) {
                sch.stopUsing(Locks.DRIVE_MOTORS)
                mecanum(y, x, rx)
            }}

        fun mecanum(y: Double, x: Double, rx: Double) {
            val botHeading: Double = hw.pinpoint.getHeading(AngleUnit.RADIANS) + PI/2

            var rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)

            rotX *= 1.1 // Counteract imperfect strafing

            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
            val frontLeftPower  = (rotY + rotX + rx) / denominator
            val backLeftPower   = (rotY - rotX + rx) / denominator
            val frontRightPower = (rotY - rotX - rx) / denominator
            val backRightPower  = (rotY + rotX - rx) / denominator

            hw.frontLeft.power  = frontLeftPower
            hw.backLeft.power   = backLeftPower
            hw.frontRight.power = frontRightPower
            hw.backRight.power  = backRightPower
        }


    }}
