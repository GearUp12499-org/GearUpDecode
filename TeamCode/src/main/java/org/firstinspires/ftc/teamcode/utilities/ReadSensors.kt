package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware

@TeleOp
class ReadSensors : LinearOpMode() {
    private fun colorSensor(label: String, cs: RevColorSensorV3) {
        telemetry.addData(label, "%.2f mm".format(cs.getDistance(DistanceUnit.MM)))
    }

    override fun runOpMode() {
        val hw = CompBot2Hardware(hardwareMap)
        waitForStart()

        val telemetry = telemetry
        telemetry.msTransmissionInterval = 10
        while (opModeIsActive()) {
            colorSensor("colorTopLeft", hw.colorTopLeft)
            colorSensor("colorTopRight", hw.colorTopRight)
            telemetry.update()
        }
    }
}
