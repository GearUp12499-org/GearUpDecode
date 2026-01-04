package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

import java.util.List;

@TeleOp
public class turretdistancetracking extends LinearOpMode {
    CompBot2Hardware hardware;

    private static final int TARGET_TAG_ID = 24;

    private static final double kP = 0.06;
    private static final double kD = 0.005;

    private static final double MAX_POWER = 0.8;
    private static final double MIN_POWER = 0.05;
    private static final double DEADBAND = 1.0;

    private static final int SOFT_LIMIT_BUFFER = 20;

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

    private double getTurretPower(double tx, double deltat) {
        if (Math.abs(tx) < DEADBAND) {
            prevError = tx;
            return 0;
        }

        double p = kP * tx;
        double d = deltat > 0 ? kD * ((tx - prevError) / deltat) : 0;
        double output = p + d;

        prevError = tx;

        if (output > 0 && output < MIN_POWER){
            output = MIN_POWER;
        }
        if (output < 0 && output > -MIN_POWER) {
            output = -MIN_POWER;
        }
        return Range.clip(output, -MAX_POWER, MAX_POWER);
    }

    private double getDistanceToGoal(double ta) {
        return Math.sqrt(56.0 / ta) - 5.82;
    }

    private void trackAprilTag() {

        LLResult result = hardware.limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            hardware.turret.setPower(0);
            prevError = 0;
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags.isEmpty()) {
            hardware.turret.setPower(0);
            prevError = 0;
            return;
        }

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == TARGET_TAG_ID) {
                target = tag;
                break;
            }
        }

        if (target == null) {
            hardware.turret.setPower(0);
            prevError = 0;
            return;
        }

        double tx = target.getTargetXDegrees();
        double ty = target.getTargetYDegrees();
        double ta = target.getTargetArea();
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double power = getTurretPower(tx, dt);
        power = limit(power, getTurretPosition());

        hardware.turret.setPower(power);

        double limelightMountAngleDegrees = 30.0;
        double limelightLensHeightInches = 13.5;
        double goalHeightInches = 29.5;
        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        telemetry.addData("distance from goal", "%.5f", getDistanceToGoal(ta));
        telemetry.addData("distance", "%.2f", distanceFromLimelightToGoalInches);
        telemetry.addData("tx", "%.5f", tx);
        telemetry.addData("ty", "%.5f", ty);
        telemetry.addData("ta","%.5f", ta);
        telemetry.addData("Turret pos", getTurretPosition());
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new CompBot2Hardware(hardwareMap);
        hardware.initMotion();

        hardware.turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hardware.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hardware.limelight.setPollRateHz(100);
        hardware.limelight.pipelineSwitch(2);
        hardware.limelight.start();

        hardware.bottomBallStop.setPosition(0.42);

        double targetvel = 1200;
        double targetpos = 0.182;

        boolean wasb = false;
        boolean wasx = false;
        boolean wasdpad = false;

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {

            trackAprilTag();

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
            telemetry.addData("Current Vel", hardware.getshoot1vel());
            telemetry.addData("At Speed", Math.abs(targetvel - hardware.getshoot1vel()) < 20);
            telemetry.update();
        }

        hardware.turret.setPower(0);
        hardware.limelight.stop();
    }
}