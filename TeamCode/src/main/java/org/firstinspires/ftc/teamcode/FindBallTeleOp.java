package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;
import java.util.Arrays;

@TeleOp
public class FindBallTeleOp extends LinearOpMode {
    private CompBotHardware Hardware;
    public Servo limelightLight1;
    public Servo limelightLight2;
    public Limelight3A Limelight;

    @Override
    public void runOpMode() {
        limelightLight1 = hardwareMap.get(Servo.class, "limelightLight1");
        limelightLight2 = hardwareMap.get(Servo.class, "limelightLight2");
        Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight.setPollRateHz(100);

        // set pipeline
        Limelight.start();
        Limelight.pipelineSwitch(6);
        telemetry.addData("Pipeline Number: ", Limelight.getStatus().getPipelineIndex());

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.x) {
                    limelightLight1.setPosition(1);
                    limelightLight2.setPosition(1);
                    // start streaming

                    LLResult result = Limelight.getLatestResult();

                    if (result == null) {
                        telemetry.addLine("Result Null");
                    } else {
                        double[] inputs = {1.0, 0.0, 0.0};
                        Limelight.updatePythonInputs(inputs);

                        double[] py = result.getPythonOutput();


                        if (py == null ) {
                            telemetry.addLine("bad python output. :(");
                        } else {
                            telemetry.addData("llpython", Arrays.toString(py));

                            double returnType = py[0];
                            double xOff = py[1];
                            double yOff = py[2];
                            double forward = py[3];
                            double angle = py[4];

                            //Error
                            if (returnType == -1.0) {
                                telemetry.addLine("error :(");
                            }

                            //None
                            else if (returnType == 0.0) {
                                telemetry.addLine("no artifact detected");
                            }

                            //Found
                            else if (returnType == 1.0) {
                                telemetry.addLine("target found!");
                                telemetry.addData("distance forward", forward);
                                telemetry.addData("xOff", xOff);
                                telemetry.addData("yOff", yOff);
                                telemetry.addData("angle", angle);
                            }
                        }
                    }

                    telemetry.update();
                }
                telemetry.update();
            }
        }
    }
