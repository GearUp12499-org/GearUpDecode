package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;

@TeleOp
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

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FrontLeftPower = (y+x+rx)/denominator;
            double FrontRightPower = (y-x-rx)/denominator;
            double BackRightPower = (y+x-rx)/denominator;
            double BackLeftPower = (y-x+rx)/denominator;

            hardware.frontLeft.setPower(FrontLeftPower);
            hardware.frontRight.setPower(FrontRightPower);
            hardware.backRight.setPower(BackRightPower);
            hardware.backLeft.setPower(BackLeftPower);


        }
    }
}
