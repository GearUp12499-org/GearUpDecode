//package org.firstinspires.ftc.teamcode
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import io.github.gearup12499.taskshark.FastScheduler
//import io.github.gearup12499.taskshark_android.TaskSharkAndroid
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
//import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
//import org.firstinspires.ftc.teamcode.systems.ShooterImpl
//import kotlin.math.PI
//import kotlin.math.abs
//import kotlin.math.cos
//import kotlin.math.max
//import kotlin.math.sin
//
//@TeleOp
//class TeleOpOld : LinearOpMode() {
//    private lateinit var hw: CompBot2Hardware
//
//    override fun runOpMode() {
//        TaskSharkAndroid.setup()
//        val sch = FastScheduler()
//        hw = CompBot2Hardware(hardwareMap)
//        hw.pinpoint.resetPosAndIMU()
//
//        val shooter = sch.add(ShooterImpl(hw))
//
//        waitForStart()
//
//        hw.initMotion()
//        hw.slider.position = CompBot2Hardware.SLIDER_IN
//
//        var wasX = false
//        var wasRB = false
//
//        while (opModeIsActive()) {
//            sch.tick()
//            hw.pinpoint.update()
//
//            val d = when {
//                gamepad1.a -> 0.25
//                gamepad1.b -> -0.25
//                else -> 0.0
//            }
//            hw.frontLeft.power = d
//            hw.frontRight.power = d
//            hw.backLeft.power = d
//            hw.backRight.power = d
//
//            val isX = gamepad1.x
//            if (isX && !wasX) shooter.setTarget(1600.0)
//            if (!isX && wasX) shooter.setTarget(0.0)
//            wasX = isX
//
//            hw.turret.power = when {
//                gamepad1.dpad_right -> 0.5
//                gamepad1.dpad_left -> -0.5
//                else -> 0.0
//            }
//
//            hw.intake.power = when {
//                gamepad1.y -> 1.0
//                else -> 0.0
//            }
//            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
//            val x = gamepad1.left_stick_x.toDouble()
//            val rx = gamepad1.right_stick_x.toDouble()
//
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                val pos = hw.pinpoint.position
//                val new = Pose2D(
//                    DistanceUnit.INCH,
//                    pos.getX(DistanceUnit.INCH),
//                    pos.getY(DistanceUnit.INCH),
//                    AngleUnit.RADIANS,
//                    0.0
//                )
//                hw.pinpoint.position = new
//            }
//
//            val isRB = gamepad1.right_bumper
//            if (isRB && !wasRB) {
////                hw.slider.position = 0.5
//                hw.slider.position = CompBot2Hardware.SLIDER_OUT
//            }
//            if (!isRB && wasRB) {
//                hw.slider.position = CompBot2Hardware.SLIDER_IN
//            }
//            wasRB = isRB
//
//            val botHeading: Double = hw.pinpoint.getHeading(AngleUnit.RADIANS) + PI / 2
//
//
//            // Rotate the movement direction counter to the bot's rotation
//            var rotX = x * cos(-botHeading) - y * sin(-botHeading)
//            val rotY = x * sin(-botHeading) + y * cos(-botHeading)
//
//            rotX *= 1.1 // Counteract imperfect strafing
//
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
//            val frontLeftPower = (rotY + rotX + rx) / denominator
//            val backLeftPower = (rotY - rotX + rx) / denominator
//            val frontRightPower = (rotY - rotX - rx) / denominator
//            val backRightPower = (rotY + rotX - rx) / denominator
//
//            hw.frontLeft.power = frontLeftPower
//            hw.backLeft.power = backLeftPower
//            hw.frontRight.power = frontRightPower
//            hw.backRight.power = backRightPower
//
//            val pos = hw.pinpoint.position
//            telemetry.addData("pinpoint status", hw.pinpoint.deviceStatus)
//            telemetry.addData(
//                "pinpoint xya", "%.2f\" %.2f\" %.2fdeg".format(
//                    pos.getX(DistanceUnit.INCH),
//                    pos.getY(DistanceUnit.INCH),
//                    pos.getHeading(AngleUnit.DEGREES)
//                )
//            )
//            telemetry.addLine("Analog Inputs (% of 3.3V)")
//            telemetry.addData(
//                "ballStopEnc",
//                hw.ballStopEncoder.voltage / hw.ballStopEncoder.maxVoltage
//            )
//            telemetry.addData("hoodEnc", hw.hoodEncoder.voltage / hw.hoodEncoder.maxVoltage)
//            telemetry.addData("sliderEnc", hw.sliderEncoder.voltage / hw.sliderEncoder.maxVoltage)
//            telemetry.addData("sliderWrite", hw.slider.position)
//            telemetry.addLine("Distance (inch)")
////            telemetry.addData("distanceRight", hw.distanceRight.getDistance(DistanceUnit.INCH))
//            telemetry.addData("distanceLeft", hw.distanceLeft.getDistance(DistanceUnit.INCH))
//            telemetry.addLine("Digital I/O")
//            telemetry.addData("frontRamp", hw.frontRamp.state)
//            telemetry.addData("middleRamp", hw.middleRamp.state)
//            telemetry.addData(
//                "color left",
//                hw.colorTopLeft.getDistance(DistanceUnit.MM)
//            )
//            telemetry.addData(
//                "color bottom l",
//                hw.colorBottomLeft.getDistance(DistanceUnit.MM)
//            )
//            telemetry.addData(
//                "color bottom r",
//                hw.colorBottomRight.getDistance(DistanceUnit.MM)
//            )
//            telemetry.update()
//        }
//    }
//}