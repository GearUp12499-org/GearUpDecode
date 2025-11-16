package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware

@TeleOp
class IndexerCheck : LinearOpMode() {
    override fun runOpMode() {
        val hw = CompBotHardware(hardwareMap)

        telemetry.msTransmissionInterval = 50
        hw.indexer.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        hw.indexer.power = 0.0

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.x) {
                hw.indexer.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                hw.indexer.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            telemetry.addData("ticks", hw.indexer.currentPosition)
            telemetry.update()
        }
    }
}