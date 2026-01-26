package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

import java.util.List;

@TeleOp
public class TurretTrackingPlus extends LinearOpMode {

    CompBot2Hardware hardware;

    private static final int TARGET_TAG_ID = 24;

    private static final double kP = 0.06;
    private static final double kI = 0.0003;
    private static final double kD = 0.0005;

    private static final double MAX_POWER = 0.8;
    private static final double MIN_POWER = 0.00;
    private static final double MAX_I = 0.2;
    private static final double DEADBAND = 1.0;

    private static final int SOFT_LIMIT_BUFFER = 10;
    private static final double IMU_HANDOFF_THRESHOLD = 30.0;

    private double prevError = 0;
    private double integralError = 0;
    private ElapsedTime loopTimer = new ElapsedTime();
    private int encoderOffset = 0;

    private boolean isLimelightTracking = false;
    private boolean resetPID = true;

    private double lastTx = 0;
    private int lastEncoderPosAtCapture = 0;
    private static final double TICKS_PER_DEGREE = (double) CompBot2Hardware.TURRET_CW_90 / 90.0;
    private static final double VELOCITY_THRESHOLD = 50;

    private double lastImuError = 0;
    private int lastImuEncoderPosAtCapture = 0;

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

    private double getTurretPower(double error, double deltat) {
        if (Math.abs(error) < DEADBAND) {
            prevError = error;
            integralError = 0;
            return 0;
        }

        if (resetPID) {
            prevError = error;
            integralError = 0;
            resetPID = false;
        }

        integralError += error * deltat;
        integralError = Range.clip(integralError, -MAX_I, MAX_I);

        double p = kP * error;
        double i = kI * integralError;
        double d = deltat > 0 ? kD * ((error - prevError) / deltat) : 0;

        double output = p + i + d;

        prevError = error;

        if (output > 0 && output < MIN_POWER) {
            output = MIN_POWER;
        } else if (output < 0 && output > -MIN_POWER) {
            output = -MIN_POWER;
        }

        return Range.clip(output, -MAX_POWER, MAX_POWER);
    }

    private double taToDistance(double ta) {
        return Math.sqrt(56.0 / ta) - 5.82;
    }

    private boolean isDestinationReachable = true;

    private double getPinpointGoalYawDiff(Pose2D goalPose, int currentTurretEncoder) {
        Pose2D robotPose = hardware.pinpoint.getPosition();

        double x = goalPose.getX(DistanceUnit.INCH) - robotPose.getX(DistanceUnit.INCH);
        double y = goalPose.getY(DistanceUnit.INCH) - robotPose.getY(DistanceUnit.INCH);
        double goalAngle = Math.atan2(y, x);
        double goalAngleDeg = AngleUnit.normalizeDegrees(goalAngle * 180 / Math.PI);

        double botHeading = AngleUnit.normalizeDegrees(robotPose.getHeading(AngleUnit.DEGREES));

        double turretRotationDeg = currentTurretEncoder / TICKS_PER_DEGREE; // How much to rotate the turret, turret is
                                                                            // backwards to the front of the robot
        double turretWorldHeading = AngleUnit.normalizeDegrees(botHeading + 180 - turretRotationDeg);

        telemetry.addData("bot heading (deg)", botHeading);
        telemetry.addData("turret rotation (deg)", turretRotationDeg);
        telemetry.addData("turret world heading (deg)", turretWorldHeading);
        telemetry.addData("goal angle (deg)", goalAngleDeg);

        double error = AngleUnit.normalizeDegrees(goalAngleDeg - turretWorldHeading);

        double limelightConventionError = -error;

        int targetTicks = currentTurretEncoder + (int) (limelightConventionError * TICKS_PER_DEGREE);
        isDestinationReachable = (targetTicks >= CompBot2Hardware.TURRET_CCW_STOP &&
                targetTicks <= CompBot2Hardware.TURRET_CW_STOP);

        return limelightConventionError;
    }

    private void turretTrackingController(Pose2D goalPose) {
        hardware.pinpoint.update();

        double dt = loopTimer.seconds();
        loopTimer.reset();

        int currentEncoder = getTurretPosition();
        double rawImuError = getPinpointGoalYawDiff(goalPose, currentEncoder);

        double refinedImuError;
        if (Math.abs(hardware.turret.getVelocity()) < VELOCITY_THRESHOLD) {
            lastImuError = rawImuError;
            lastImuEncoderPosAtCapture = currentEncoder;
            refinedImuError = rawImuError;
        } else {
            double deltaTicks = currentEncoder - lastImuEncoderPosAtCapture;
            refinedImuError = lastImuError - (deltaTicks / TICKS_PER_DEGREE);
        }

        LLResult result = hardware.limelight.getLatestResult();
        double refinedLimelightError = Double.NaN;
        boolean limelightVisible = false;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            LLResultTypes.FiducialResult target = null;
            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_TAG_ID) {
                    target = tag;
                    break;
                }
            }
            if (target != null) {
                limelightVisible = true;
                if (Math.abs(hardware.turret.getVelocity()) < VELOCITY_THRESHOLD) {
                    lastTx = target.getTargetXDegrees();
                    lastEncoderPosAtCapture = currentEncoder;
                    refinedLimelightError = lastTx;
                } else {
                    double deltaTicks = currentEncoder - lastEncoderPosAtCapture;
                    refinedLimelightError = lastTx - (deltaTicks / TICKS_PER_DEGREE);
                }
            }
        }

        boolean nextIsLimelight = limelightVisible;

        if (nextIsLimelight != isLimelightTracking) {
            isLimelightTracking = nextIsLimelight;
            resetPID = true;
        }

        double finalError = isLimelightTracking ? refinedLimelightError : refinedImuError;

        double power = getTurretPower(finalError, dt);
        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        power = limit(power, currentEncoder);

        hardware.turret.setPower(power);

        telemetry.addData(">>> MODE", isLimelightTracking ? "LIMELIGHT" : "IMU");
        telemetry.addData("error", "%.2f", finalError);
        telemetry.addData("power", "%.3f", power);
        telemetry.addData("reachable", isDestinationReachable);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new CompBot2Hardware(hardwareMap);
        hardware.initMotion();

        hardware.turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hardware.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hardware.limelight.setPollRateHz(150);
        hardware.limelight.pipelineSwitch(2);
        hardware.limelight.start();

        hardware.bottomBallStop.setPosition(0.42);

        double targetvel = 1200;
        double targetpos = 0.182;

        boolean wasb = false;
        boolean wasx = false;
        boolean wasdpad = false;

        waitForStart();

        hardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -180));

        loopTimer.reset();

        Pose2D goalPose = new Pose2D(DistanceUnit.INCH, 58, -56, AngleUnit.DEGREES, -2.318 * 180 / Math.PI);

        while (opModeIsActive()) {

            turretTrackingController(goalPose);

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
            telemetry.addData("Current Vel", hardware.getShoot1Vel());
            telemetry.addData("At Speed", Math.abs(targetvel - hardware.getShoot1Vel()) < 20);
            telemetry.update();
        }

        hardware.turret.setPower(0);
        hardware.limelight.stop();
    }
}