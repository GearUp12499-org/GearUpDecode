package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

import java.util.List;

@TeleOp
public class ATTurret extends LinearOpMode {

    // LL
    private static final int TARGET_TAG_ID = 20;
    private static final int PIPELINE = 6;
    private static final int POLL_RATE = 100;

    // PD no I
    private static final double kP = 0.03;
    private static final double kD = 0.005;

    private static final double MAX_POWER = 0.6;
    private static final double MIN_POWER = 0.05;
    private static final double DEADBAND = 1.0;

    // adjust
    private static final int SOFT_LIMIT_BUFFER = 30;

    private Limelight3A limelight;
    private DcMotorEx turretMotor;
    private double prevError = 0;
    private ElapsedTime loopTimer = new ElapsedTime();
    private int encoderOffset = 0;

    private int getTurretPosition() {
        return turretMotor.getCurrentPosition() - encoderOffset;
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

    private double getNewTurretPower(double tX, double deltaTime) {
        double error = tX;

        if (Math.abs(error) < DEADBAND) {
            prevError = error;
            return 0;
        }

        double pTerm = kP * error;

        double dTerm = 0;
        if (deltaTime > 0) {
            double errorRate = (error - prevError) / deltaTime;
            dTerm = kD * errorRate;
        }

        prevError = error;

        double output = pTerm + dTerm;

        if (output > 0 && output < MIN_POWER) {
            output = MIN_POWER;
        } else if (output < 0 && output > -MIN_POWER) {
            output = -MIN_POWER;
        }

        output = Range.clip(output, -MAX_POWER, MAX_POWER);

        return output;
    }

    private boolean trackAprilTag(int tagId) {
        LLResult result = limelight.getLatestResult();
        int currentPosition = getTurretPosition();

        if (result == null || !result.isValid()) {
            telemetry.addData("ll", "No valid result");
            turretMotor.setPower(0);
            prevError = 0;
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials.isEmpty()) {
            telemetry.addData("tags", "None detected");
            turretMotor.setPower(0);
            prevError = 0;
            return false;
        }

        LLResultTypes.FiducialResult targetTag = null;

        for (LLResultTypes.FiducialResult tag : fiducials) {
            if (tagId == -1 || tag.getFiducialId() == tagId) {
                targetTag = tag;
                break;
            }
        }

        if (targetTag == null) {
            telemetry.addData("tag " + tagId, "Not visible");
            telemetry.addData("tags", fiducials.size());
            turretMotor.setPower(0);
            prevError = 0;
            return false;
        }

        double tX = targetTag.getTargetXDegrees();

        double deltaTime = loopTimer.seconds();
        loopTimer.reset();

        double power = getNewTurretPower(tX, deltaTime);
        power = limit(power, currentPosition);
        turretMotor.setPower(power);

        telemetry.addData("id", targetTag.getFiducialId());
        telemetry.addData("tx (degrees)", "%.2f", tX);
        telemetry.addData("pos", "%.1f°", currentPosition);
        telemetry.addData("power", "%.3f", power);

        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            telemetry.addData("botpose", "(%.2f, %.2f)", botpose.getPosition().x, botpose.getPosition().y);
        }

        return true;
    }

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoderOffset = 0;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(POLL_RATE);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addData("pipeline", limelight.getStatus().getPipelineIndex());
        telemetry.addData("tracking", TARGET_TAG_ID);
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            boolean isTracking = trackAprilTag(TARGET_TAG_ID);
            int currentPosition = getTurretPosition();

            telemetry.addData("status", isTracking ? "ic" : "i dont c");
            telemetry.addData("ticks", "%d (limit: %d to %d)", currentPosition, CompBot2Hardware.TURRET_CCW_STOP,
                    CompBot2Hardware.TURRET_CW_STOP);

            if (gamepad1.a) { // reset
                turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                encoderOffset = 0;
            }

            telemetry.update();
        }

        turretMotor.setPower(0);
        limelight.stop();
    }
}