package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp

public class ShooterTest2 extends LinearOpMode {

    CompBot2Hardware hardware;

    @Override

    public void runOpMode() throws InterruptedException {
        hardware = new CompBot2Hardware(hardwareMap);

        hardware.initMotion();

        hardware.limelight.setPollRateHz(100);
        hardware.limelight.pipelineSwitch(2);
        hardware.limelight.start();

        hardware.bottomBallStop.setPosition(0.42);

        boolean wasb = false;
        boolean wasx = false;
        boolean wasdpad = false;

      //  double ticks_per_degree = TURRET_CW_90 / 90.0;
        double targetvel = 1200;
        double targetpos = 0.182;

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = hardware.limelight.getLatestResult();

            hardware.copyShooterPower();

            if (gamepad1.dpad_up) {
                hardware.flipper.setPosition(0.68);
                sleep(1000);
                hardware.setIntakePower(-0.8);

                hardware.flipper.setPosition(0.25);
                sleep(500);
                hardware.setIntakePower(1);
            }
            if (gamepad1.dpad_right && !wasdpad) {
                targetpos += 0.094;

                if (targetpos > 0.56) {
                    targetpos = 0.182;
                }
                hardware.hood.setPosition(targetpos);
            }

            if (gamepad1.dpad_down) {
                hardware.bottomBallStop.setPosition(0.58);
            }

            if (gamepad1.dpad_left) {
                hardware.bottomBallStop.setPosition(0.15);
                sleep(500);
                hardware.flipper.setPosition(0.68);
                sleep(800);
                hardware.setIntakePower(-0.8);

                hardware.flipper.setPosition(0.25);
                sleep(500);
                hardware.setIntakePower(1);
            }
            if (gamepad1.a) {
                hardware.setShoot1Vel(targetvel);
                hardware.setIntakePower(1);
            }

            if (gamepad1.b && !wasb) {
                targetvel += 20;
                hardware.setShoot1Vel(targetvel);
            }

            if (gamepad1.x && !wasx) {
                targetvel -= 20;
                hardware.setShoot1Vel(targetvel);
            }

            if (gamepad1.y) {
                hardware.setShoot1Vel(0);
                hardware.setIntakePower(0);
            }

            wasb = gamepad1.b;
            wasx = gamepad1.x;
            wasdpad = gamepad1.dpad_right;

            double currentVel = hardware.getShoot1Vel();
            double hoodpos = hardware.gethoodpos();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            if (result != null ) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
            }
            telemetry.addData("target velocity", targetvel);
            telemetry.addData("Current Vel: ", currentVel);
            telemetry.addData("at target", Math.abs(targetvel - currentVel) < 20);
            telemetry.addData("hood position", hoodpos);
            telemetry.update();


        }
    }
}