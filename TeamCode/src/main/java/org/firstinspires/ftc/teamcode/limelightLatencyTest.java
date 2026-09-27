package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

@TeleOp
public class limelightLatencyTest extends LinearOpMode {

    CompBot2Hardware hw;

    @Override
    public void runOpMode() throws InterruptedException {

        hw = new CompBot2Hardware(hardwareMap);

        hw.limelight.start();
        hw.limelight.pipelineSwitch(7);

        waitForStart();

        while (opModeIsActive()){

            double time1 = System.nanoTime();
            LLResult result = hw.limelight.getLatestResult();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double timeInLimelight = captureLatency + targetingLatency;

            double arrivalTime = result.getTimestamp();
            double currentTime = System.currentTimeMillis();
            double timeSittingOnHub = currentTime-arrivalTime;

            Log.i("totalTime", String.valueOf(timeInLimelight+timeSittingOnHub));

        }
    }
}
