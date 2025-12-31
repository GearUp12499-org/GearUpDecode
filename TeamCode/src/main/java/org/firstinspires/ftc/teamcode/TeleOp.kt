package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware

@TeleOp
class TeleOp : LinearOpMode() {
    private lateinit var hw: CompBot2Hardware

    override fun runOpMode() {
        hw = CompBot2Hardware(hardwareMap)
        hw.pinpoint.resetPosAndIMU()

        waitForStart()
        while (opModeIsActive()) {
            hw.pinpoint.update()

            val d = when {
                gamepad1.a -> 0.25
                gamepad1.b -> -0.25
                else -> 0.0
            }
            hw.frontLeft.power = d
            hw.frontRight.power = d
            hw.backLeft.power = d
            hw.backRight.power = d

            val s = when {
                gamepad1.x -> 0.25
                else -> 0.0
            }
            hw.shoot1.power = s
            hw.shoot2.power = s

            hw.turret.power = when {
                gamepad1.dpad_right -> 0.5
                gamepad1.dpad_left -> -0.5
                else -> 0.0
            }

            hw.intake.power = when {
                gamepad1.y -> 0.5
                else -> 0.0
            }

            val pos = hw.pinpoint.position
            telemetry.addData("pinpoint status", hw.pinpoint.deviceStatus)
            telemetry.addData(
                "pinpoint xya", "%.2f\" %.2f\" %.2fdeg".format(
                    pos.getX(DistanceUnit.INCH),
                    pos.getY(DistanceUnit.INCH),
                    pos.getHeading(AngleUnit.DEGREES)
                )
            )
            telemetry.addLine("Analog Inputs (% of 3.3V)")
            telemetry.addData("ballStopEnc", hw.ballStopEncoder.voltage/hw.ballStopEncoder.maxVoltage)
            telemetry.addData("hoodEnc", hw.hoodEncoder.voltage/hw.hoodEncoder.maxVoltage)
            telemetry.addData("sliderEnc", hw.sliderEncoder.voltage/hw.sliderEncoder.maxVoltage)
            telemetry.addData("dropDownEnc", hw.dropDownEncoder.voltage/hw.dropDownEncoder.maxVoltage)
            telemetry.addLine("Distance (inch)")
            telemetry.addData("distanceRight", hw.distanceRight.getDistance(DistanceUnit.INCH))
            telemetry.addData("distanceLeft", hw.distanceLeft.getDistance(DistanceUnit.INCH))
            telemetry.addLine("Digital I/O")
            telemetry.addData("frontRamp", hw.frontRamp.state)
            telemetry.addData("middleRamp", hw.middleRamp.state)
            telemetry.update()
        }
    }
}