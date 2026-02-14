package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

import java.util.List;

@TeleOp

public class LimeLightTeleOp extends LinearOpMode {
    final double inchespermeter = 39.37;
    final double coordflip = -1;
    private Limelight3A limelight3A;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.setPollRateHz(100);

        limelight3A.start();

waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight3A.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {

                    telemetry.addData("Z", botpose.getPosition().z*inchespermeter);
                    telemetry.addData("X offset", botpose.getPosition().x*inchespermeter*coordflip);
                    telemetry.addData("y offset", botpose.getPosition().y*inchespermeter*coordflip);
                } else {
                    telemetry.addLine("null");
                }
            }
            telemetry.update();
        }
    }
}