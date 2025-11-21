package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;
import org.firstinspires.ftc.teamcode.hardware.FileUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Locale;

@TeleOp
public class ShooterPIDTuning extends LinearOpMode {

    CompBotHardware hardware;

    @Override
    public void runOpMode() {

        hardware = new CompBotHardware(hardwareMap);

        double targetVel = 1200;

        double currentVel = 0;

        boolean atTarget = false;

        double secondsAtTarget = 0;

        double newP = 1000;
        double newI = 3;
        double newD = 0.25;
        double newF = 0;

        waitForStart();

        ElapsedTime secsAtTarget = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        PIDFCoefficients oldPID = hardware.shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients newPID = new PIDFCoefficients(newP, newI, newD, newF);

        hardware.shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPID);
        hardware.shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients currentPID = hardware.shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("P,I,D, F(new)", "%.04f, %.04f, %.04f, %.04f",
                currentPID.p, currentPID.i, currentPID.d, currentPID.f);
        telemetry.addData("target velocity", targetVel);
        telemetry.addData("current velocity", currentVel);

        telemetry.update();


        while (opModeIsActive()) {

            if (gamepad1.left_bumper){
                newP += 0.001;
            }
            if (gamepad1.right_bumper){
                newP -= 0.001;
            }
            if (gamepad1.a) {
                ArrayList<Long> Time = new ArrayList<>();
                ArrayList<Double> Velocity = new ArrayList<>();

                double startTime = runtime.time();
                double deltaTime = 0;

                hardware.shooter1.setVelocity(targetVel);

                secsAtTarget.reset();

                while (secondsAtTarget < 2 && deltaTime<10) {
                    secondsAtTarget = secsAtTarget.time();
                    currentVel = hardware.shooter1.getVelocity();
                    if (Math.abs(currentVel - targetVel) > 45) {
                        secsAtTarget.reset();
                    }

                    telemetry.addData("P,I,D, F(new)", "%.04f, %.04f, %.0f, %.04f",
                            currentPID.p, currentPID.i, currentPID.d, currentPID.f);
                    telemetry.addData("target velocity", targetVel);
                    telemetry.addData("current velocity", currentVel);

                    telemetry.update();

                    double endTime = runtime.time();
                    deltaTime = endTime - startTime;

                    Time.add(System.nanoTime());
                    Velocity.add(currentVel);
                }
                hardware.shooter1.setVelocity(0);

                double endTime = runtime.time();
                deltaTime = endTime - startTime;

                telemetry.addData("time to target", deltaTime);
                telemetry.update();


                File f = FileUtil.getfile();
                RobotLog.i("Writing file to " + f);
                try (FileOutputStream fos = new FileOutputStream(f);
                     OutputStreamWriter writer = new OutputStreamWriter(fos, StandardCharsets.UTF_8)
                ) {
                    writer.write("time,velocity\n");
                    for (int i = 0; i < Time.size(); i++) {
                        writer.write(String.format(
                                Locale.ROOT,
                                "%d,%f\n",
                                Time.get(i),
                                Velocity.get(i)

                        ));
                    }
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }

            }
        }
    }
}
