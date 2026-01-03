package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

@TeleOp

public class ShooterTest2 extends LinearOpMode {

    CompBot2Hardware hardware;

    @Override

    public void runOpMode() throws InterruptedException {
        hardware = new CompBot2Hardware(hardwareMap);

        hardware.initMotion();

        boolean wasb = false;
        boolean wasx = false;
        boolean wasdpad = false;

        double targetvel = 1200;
        double targetpos = 0.182;

        waitForStart();

        while (opModeIsActive()) {

            hardware.copyShooterPower();

            if(gamepad1.dpad_up){
                hardware.flipper.setPosition(0.68);
                sleep(800);
                hardware.intake.setPower(-0.8);

                hardware.flipper.setPosition(0.25);
                sleep(500);
                hardware.intake.setPower(1);
            }
            if (gamepad1.dpad_right && !wasdpad) {
                targetpos += 0.094;

                if (targetpos > 0.56) {
                    targetpos = 0.182;
                }
                hardware.hood.setPosition(targetpos);
            }

            if(gamepad1.a){
                hardware.setShoot1Vel(targetvel);
                hardware.intake.setPower(1);
            }

            if (gamepad1.b && !wasb){
                targetvel += 20;
                hardware.setShoot1Vel(targetvel);
            }

            if(gamepad1.x && !wasx) {
                targetvel -= 20;
                hardware.setShoot1Vel(targetvel);
            }

            if (gamepad1.y){
                hardware.setShoot1Vel(0);
                hardware.intake.setPower(0);
            }

            wasb = gamepad1.b;
            wasx = gamepad1.x;
            wasdpad = gamepad1.dpad_right;
            double currentVel = hardware.getshoot1vel();
            double hoodpos = hardware. gethoodpos();
            telemetry.addData("target velocity", targetvel);
            telemetry.addData("Current Vel: ", currentVel);
            telemetry.addData("at target", Math.abs(targetvel- currentVel)<20);
            telemetry.addData("hood position", hoodpos);
            telemetry.update();
        }
    }
}