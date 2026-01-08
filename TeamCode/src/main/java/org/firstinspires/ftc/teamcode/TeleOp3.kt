package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.Locks
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_MID_RANGE
import org.firstinspires.ftc.teamcode.systems.Combo
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.tasks.PinpointTask
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.tasks.stopUsing
import org.firstinspires.ftc.teamcode.utilities.StaticStore
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.time.Duration.Companion.seconds

abstract class TeleOp3(red: Boolean) : LinearOpMode() {
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE

    private lateinit var hw: CompBot2Hardware
    private lateinit var shooter: ShooterImpl
    private lateinit var scheduler: FastScheduler

    override fun runOpMode() {
        hw = CompBot2Hardware(hardwareMap)
        scheduler = FastScheduler()

        if (StaticStore.duration() > 30.seconds) hw.pinpoint.resetPosAndIMU()

        // Background tasks
        scheduler.add(PinpointTask(hw.pinpoint))
        val robotStartTask = scheduler.add(SentinelTask())
        shooter = robotStartTask.then(ShooterImpl(hw))
        robotStartTask.then(DriveTask())
        robotStartTask.then(OneShot {
            hw.slider.position = CompBot2Hardware.SLIDER_OUT
            hw.flipper.position = CompBot2Hardware.FLIPPER_DOWN
            hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
            hw.dropDown.position = CompBot2Hardware.DROP_DOWN_SWEET_SPOT

            hw.turret.targetPosition = 0
            hw.turret.mode = DcMotor.RunMode.RUN_TO_POSITION
            hw.turret.power = 1.0
        })

        while (opModeInInit()) {
            scheduler.tick()
        }
        robotStartTask.requestStart()
        while (opModeIsActive()) {
            scheduler.tick()
        }
    }

    private inner class DriveTask : Task<DriveTask>() {
        private inline val sch: Scheduler get() = super.scheduler!!

        override fun onTick(): Boolean {
            val sch = sch

            mecanumDispatcher(sch)
            inOut(sch)
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
            }

            if (gamepad1.backWasPressed()) {
                sch.stopUsing(Locks.DRIVE_MOTORS)

                val pos = hw.pinpoint.position
                val new = Pose2D(
                    DistanceUnit.INCH,
                    pos.getX(DistanceUnit.INCH),
                    pos.getY(DistanceUnit.INCH),
                    AngleUnit.RADIANS,
                    0.0
                )
                hw.pinpoint.position = new
            }
        }

        fun mecanum(y: Double, x: Double, rx: Double) {
            val botHeading: Double = hw.pinpoint.getHeading(AngleUnit.RADIANS) + PI / 2

            // Rotate the movement direction counter to the bot's rotation
            var rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)

            rotX *= 1.1 // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
            val frontLeftPower = (rotY + rotX + rx) / denominator
            val backLeftPower = (rotY - rotX + rx) / denominator
            val frontRightPower = (rotY - rotX - rx) / denominator
            val backRightPower = (rotY + rotX - rx) / denominator

            hw.frontLeft.power = frontLeftPower
            hw.backLeft.power = backLeftPower
            hw.frontRight.power = frontRightPower
            hw.backRight.power = backRightPower
        }

        private var gp1RB = false
        private var gp1LB = false
        private var gp1X = false

        fun inOut(sch: Scheduler) {
            val rb = gamepad1.right_bumper
            val lb = gamepad1.left_bumper
            val x = gamepad1.x

            if (rb && !gp1RB) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                sch.add(Combo.intake(hw))
            }
            if (x && !gp1X) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                sch.stopUsing(Locks.DRIVE_MOTORS)
                sch.add(VirtualGroup {
                    add(REmover.drive2Pose2(hw, poseSet.midShoot))
                    add(shooter.setTargetAndWait(SHOOT_MID_RANGE, 0.2))
                    add(OneShot {
                        hw.hood.position = CompBot2Hardware.HOOD_50
                    })
                }).then(Combo.shoot(hw))
                    .then(shooter.setTargetAsync(0.0))
            }
            if (lb && !gp1LB) {
                sch.stopUsing(Locks.INTAKE_STORAGE)
                sch.stopUsing(Locks.DRIVE_MOTORS)
                shooter.setTarget(0.0)
            }

            gp1RB = rb
            gp1LB = lb
            gp1X = x
        }
    }
}