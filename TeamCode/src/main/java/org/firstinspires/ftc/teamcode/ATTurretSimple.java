package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

import java.util.List;


@Disabled
@TeleOp
public class ATTurretSimple extends LinearOpMode {

    private static final int TARGET_TAG_ID = 24;
    private static final int PIPELINE = 2;
    private static final int POLL_RATE = 100;

    private static final double TURRET_POWER = 0.8;
    private static final int SOFT_LIMIT_BUFFER = 20;

    private static final double TICKS_PER_DEGREE = CompBot2Hardware.TURRET_CW_90 / 90.0; // approx 2.55

    private Limelight3A limelight;
    private DcMotorEx turretMotor;

    private Double getBearing(int tagId) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials.isEmpty()) {
            return null;
        }

        for (LLResultTypes.FiducialResult tag : fiducials) {
            if (tagId == -1 || tag.getFiducialId() == tagId) {
                return tag.getTargetXDegrees();
            }
        }

        return null;
    }

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_POWER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(POLL_RATE);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addData("pipeline", limelight.getStatus().getPipelineIndex());
        telemetry.addData("tracking", TARGET_TAG_ID);
        telemetry.addData("ticks/deg", "%.3f", TICKS_PER_DEGREE);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Double tx = getBearing(TARGET_TAG_ID);
            int currentPosition = turretMotor.getCurrentPosition();

            if (tx != null) {
                int turretTarget = currentPosition + (int) (TICKS_PER_DEGREE * tx);

                if (turretTarget > CompBot2Hardware.TURRET_CW_STOP - SOFT_LIMIT_BUFFER) {
                    turretTarget = CompBot2Hardware.TURRET_CW_STOP - SOFT_LIMIT_BUFFER;
                }
                if (turretTarget < CompBot2Hardware.TURRET_CCW_STOP + SOFT_LIMIT_BUFFER) {
                    turretTarget = CompBot2Hardware.TURRET_CCW_STOP + SOFT_LIMIT_BUFFER;
                }

                turretMotor.setTargetPosition(turretTarget);

                telemetry.addData("status", "TRACKING");
                telemetry.addData("tx", "%.2f°", tx);
                telemetry.addData("target", turretTarget);
            } else {
                telemetry.addData("status", "NO TAG");
            }

            telemetry.addData("position", currentPosition);
            telemetry.addData("limits", "%d to %d",
                    CompBot2Hardware.TURRET_CCW_STOP,
                    CompBot2Hardware.TURRET_CW_STOP);

            if (gamepad1.a) {
                turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setTargetPosition(0);
                turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(TURRET_POWER);
            }

            telemetry.update();
        }

        turretMotor.setPower(0);
        limelight.stop();
    }
}
