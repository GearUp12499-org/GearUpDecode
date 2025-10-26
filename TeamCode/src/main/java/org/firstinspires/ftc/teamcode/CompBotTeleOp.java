package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;

public class CompBotTeleOp extends LinearOpMode {

    CompBotHardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new CompBotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double FrontLeftPower = (y+x+rx);
            double FrontRightPower = (y-x-rx);
            double BackRightPower = (y+x-rx);
            double BackLeftPower = (y-x+rx);


        }
    }
}
