package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

@TeleOp
public class TurretTest extends LinearOpMode {

    CompBot2Hardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new CompBot2Hardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double currentPos = hardware.turretEncoder.getCurrentPosition();
            telemetry.addData("position", currentPos);
            telemetry.update();
        }

    }
}
