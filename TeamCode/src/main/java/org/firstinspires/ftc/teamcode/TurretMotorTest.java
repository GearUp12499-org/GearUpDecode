package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

import java.util.List;

@TeleOp
public class TurretMotorTest extends LinearOpMode {

    CompBot2Hardware hardware;

    private static final int TARGET_TAG_ID = 24;

    private static final double kP = 0.06;
    private static final double kD = 0.005;

    private static final double MAX_POWER = 0.8;
    private static final double MIN_POWER = 0.05;
    private static final double DEADBAND = 1.0;

    private static final int SOFT_LIMIT_BUFFER = 30;

    private double prevError = 0;
    private ElapsedTime loopTimer = new ElapsedTime();
    private int encoderOffset = 0;


    private int getTurretPosition() {
        return hardware.turret.getCurrentPosition() - encoderOffset;
    }

    private double limit(double power, int currentPosition) {
        if (currentPosition >= CompBot2Hardware.TURRET_CW_STOP - SOFT_LIMIT_BUFFER && power > 0) {
            double distanceToLimit = CompBot2Hardware.TURRET_CW_STOP - currentPosition;
            double scaleFactor = distanceToLimit / SOFT_LIMIT_BUFFER;
            power = power * Math.max(0, scaleFactor);
        }

        if (currentPosition <= CompBot2Hardware.TURRET_CCW_STOP + SOFT_LIMIT_BUFFER && power < 0) {
            double distanceToLimit = currentPosition - CompBot2Hardware.TURRET_CCW_STOP;
            double scaleFactor = distanceToLimit / SOFT_LIMIT_BUFFER;
            power = power * Math.max(0, scaleFactor);
        }

        if (currentPosition >= CompBot2Hardware.TURRET_CW_STOP && power > 0) {
            return 0;
        }
        if (currentPosition <= CompBot2Hardware.TURRET_CCW_STOP && power < 0) {
            return 0;
        }

        return power;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new CompBot2Hardware(hardwareMap);
        hardware.initMotion();

        hardware.turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hardware.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        hardware.bottomBallStop.setPosition(0.42);

        double targetvel = 1200;
        double targetpos = 0.182;

        boolean wasb = false;
        boolean wasx = false;
        boolean wasdpad = false;

        waitForStart();
        loopTimer.reset();
        int counter = 0;

        while (opModeIsActive()) {
            double power = 0.05;
            power = limit(power, getTurretPosition());
            hardware.turret.setPower(power);

            hardware.copyShooterPower();

            if (gamepad1.a) {
                hardware.setShoot1Vel(targetvel);
                hardware.intake.setPower(1);
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
                hardware.intake.setPower(0);
            }

            wasb = gamepad1.b;
            wasx = gamepad1.x;
            wasdpad = gamepad1.dpad_right;

            telemetry.addData("Target Vel", targetvel);
            telemetry.update();
            counter++;
        }

        hardware.turret.setPower(0);
        hardware.limelight.stop();
    }
}