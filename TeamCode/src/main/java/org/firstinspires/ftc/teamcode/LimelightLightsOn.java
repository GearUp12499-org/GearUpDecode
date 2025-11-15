package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;

@TeleOp
public class LimelightLightsOn extends LinearOpMode {
    CompBotHardware hardware;

    public void runOpMode() {
        hardware = new CompBotHardware(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            Servo limelightLight1 = hardware.limelightLight1;
            Servo limelightLight2 = hardware.limelightLight2;

            hardware.limelightLight1.setPosition(1.0);
            hardware.limelightLight2.setPosition(1.0);
        }
    }
}


