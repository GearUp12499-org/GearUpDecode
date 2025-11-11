package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;
import org.firstinspires.ftc.teamcode.hardware.FileUtil;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Locale;

@TeleOp
public class CompBotTeleOp extends LinearOpMode {

    public static final int ticksPerRotation = 970;
    public static final int ticksPerStep = ticksPerRotation / 6;
    CompBotHardware hardware;
    private ElapsedTime runtime;

    @Override
    public void runOpMode() {
        hardware = new CompBotHardware(hardwareMap);


        hardware.pinpoint.setOffsets(-3.9, -3.875, DistanceUnit.INCH);
        hardware.pinpoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        hardware.pinpoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.REVERSED, GoBildaPinpoint2Driver.EncoderDirection.FORWARD);
        hardware.pinpoint.recalibrateIMU();


        telemetry.addData("pose", hardware.pinpoint.getPosition());
        telemetry.update();

        waitForStart();
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        hardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,-16,AngleUnit.RADIANS, -Math.PI));



        while (opModeIsActive()) {
            hardware.pinpoint.update();

            Pose2D currentPose = hardware.pinpoint.getPosition();

            telemetry.addData("pinpointa", currentPose.getHeading(AngleUnit.RADIANS));
            telemetry.addData("pinpointx", currentPose.getX(DistanceUnit.INCH));
            telemetry.addData("pinpointy", currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("pinpoint", hardware.pinpoint.getDeviceStatus());

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double heading = currentPose.getHeading(AngleUnit.RADIANS) + Math.PI / 2;
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad1.x) {
                drive2Pose(new double[]{0, -2, 0});
                sleep(100);
                drive2Pose(new double[]{4,-2,0});
            }

            if (gamepad1.y) {
                hardware.intake.setPower(1);
            } else {
                hardware.intake.setPower(0);
            }

            if (gamepad1.a) {
                drive2Pose(CompBotHardware.shootPos);
            }

            if (gamepad1.b) {
                hardware.frontRight.setPower(0.15);
                hardware.frontLeft.setPower(0.15);
                hardware.backRight.setPower(0.15);
                hardware.backLeft.setPower(0.15);
            }

            if (gamepad2.b) {
                hardware.flipper.setPosition(CompBotHardware.FLIPPER_UP);
            } else {
                hardware.flipper.setPosition(CompBotHardware.FLIPPER_DOWN);
            }

            if (gamepad1.dpad_up) {
                hardware.indexer.setTargetPosition(hardware.indexer.getCurrentPosition() + ticksPerStep);
                hardware.indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.indexer.setPower(0.5);
            }

            if (gamepad2.right_bumper) {
                // 2000, 1500: too fast for mid range
                // 1000: too slow for mid range
                // 600, 800 too slow barely goes anywhere
                // 1200 seems good for shootingPos in hardware
                hardware.shooter1.setVelocity(1200);
            } else if (gamepad2.left_bumper) {
                hardware.shooter1.setVelocity(-500);
            } else {
                hardware.shooter1.setVelocity(0);
            }

            if (gamepad1.start) {
                spindexer0();
            }
//            hardware.frontLeft.setPower(frontLeftPower);
//            hardware.frontRight.setPower(frontRightPower);
//            hardware.backRight.setPower(backRightPower);
//            hardware.backLeft.setPower(backLeftPower);

            telemetry.addData("indexerPos", hardware.indexer.getCurrentPosition());

            telemetry.update();

        }
    }


    public void drive2Pose(double[] xya) {
        ArrayList<Long> Time = new ArrayList<>();
        ArrayList<Double> VelocityX = new ArrayList<>();
        ArrayList<Double> VelocityY = new ArrayList<>();

        ArrayList<Double> FLspeed = new ArrayList<>();
        ArrayList<Double> BLspeed = new ArrayList<>();
        ArrayList<Double> FRspeed = new ArrayList<>();
        ArrayList<Double> BRspeed = new ArrayList<>();

        ArrayList<Double> FLpower = new ArrayList<>();
        ArrayList<Double> BLpower = new ArrayList<>();
        ArrayList<Double> FRpower = new ArrayList<>();
        ArrayList<Double> BRpower = new ArrayList<>();

        ArrayList<Double> LoopTime = new ArrayList<>();
        ArrayList<Double> deltaX = new ArrayList<>();
        ArrayList<Double> deltaY = new ArrayList<>();
        ArrayList<Double> Angle = new ArrayList<>();
//        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0)); //Do we want to do this again?
//        hardware.PinPoint.setOffsets(3.4,1, DistanceUnit.INCH);
//        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

//        hardware.PinPoint.resetPosAndIMU();
//        hardware.PinPoint.recalibrateIMU();
//        hardware = new DumbledoreHardware(hardwareMap);

        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        double kp = 0.2;
        double kd = 50;


        double currenTime = runtime.time();
        double prevTime = runtime.time();
        double deltaTime = 0;
        double prevDeltaAll = 0;


        double tgtx = xya[0];
        double tgty = xya[1];
        double tgta = xya[2];


        while (true) {
            currenTime = runtime.time();

            double Timeout = timeout.time();

            hardware.pinpoint.update();

            double yVelocity = hardware.pinpoint.getVelY(DistanceUnit.INCH);
            double xVelocity = hardware.pinpoint.getVelX(DistanceUnit.INCH);

            double speed = Math.sqrt((yVelocity * yVelocity) + (xVelocity * xVelocity));

            Pose2D currentPose = hardware.pinpoint.getPosition();

            double currentx = currentPose.getX(DistanceUnit.INCH);
            double currenty = currentPose.getY(DistanceUnit.INCH);
            double currentTheta = currentPose.getHeading(AngleUnit.RADIANS);

            double deltax = tgtx - currentx;
            double deltay = tgty - currenty;
            double deltaA = tgta - currentTheta;
            deltaA = deltaA % (2 * (Math.PI));
            if (deltaA > Math.PI) {
                deltaA -= 2 * Math.PI;
            } else if (deltaA < -Math.PI) {
                deltaA += 2 * Math.PI;
            }

            telemetry.addData("delta A", deltaA);

            if (Math.abs(deltax) < 0.5 && Math.abs(deltay) < 0.5 && Math.abs(deltaA) < Math.PI / 24 && speed < 10 || Timeout > 1) {
                hardware.frontLeft.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backRight.setPower(0);
                break;
            }

            double R = 7.66;
            double F = Math.cos(currentTheta) * deltax + Math.sin(currentTheta) * deltay;
            double S = Math.sin(currentTheta) * deltax - Math.cos(currentTheta) * deltay;
            double W = R * deltaA;
            double deltaAll = Math.sqrt((F * F) + (S * S) + (W * W));

            if (Math.abs(deltaAll - prevDeltaAll) > 0.5) {
                timeout.reset();
            }

            double DFL = F + S - W;
            double DBL = F - S - W;
            double DFR = F - S + W;
            double DBR = F + S + W;

            //rescale the four speeds so the largest is +/- 1
            double tempMax1 = Math.max(Math.abs(DFL), Math.abs(DBL));
            double tempMax2 = Math.max(Math.abs(DFR), Math.abs(DBR));
            double scale = Math.max(tempMax1, tempMax2);

            if (scale < 0.01) {
                scale = 0.01;
            }

//            double scale = Math.max(Math.abs(F) + Math.abs(S) + Math.abs(W), kp*deltaAll);
            DFL /= scale;
            DBL /= scale;
            DFR /= scale;
            DBR /= scale;

            //PID computation

            deltaTime = Math.max(currenTime - prevTime, 0.001);//making sure we don't divide by 0

            double pid = kp * deltaAll + kd * (deltaAll - prevDeltaAll) / deltaTime;

            //if pid<1, rescale so fastest speed is +/- pid
            if (Math.abs(pid) < 1) {
                DFL *= pid;
                DBL *= pid;
                DFR *= pid;
                DBR *= pid;
            }

            double PFL = speed2Power(DFL);
            double PFR = speed2Power(DFR);
            double PBL = speed2Power(DBL);
            double PBR = speed2Power(DBR);

            hardware.frontLeft.setPower(PFL);
            hardware.backLeft.setPower(PBL);
            hardware.frontRight.setPower(PFR);
            hardware.backRight.setPower(PBR);

            Time.add(System.nanoTime());
            VelocityY.add(yVelocity);
            VelocityX.add(xVelocity);
            FLspeed.add(DFL);
            FRspeed.add(DFR);
            BLspeed.add(DBL);
            BRspeed.add(DBR);

            FLpower.add(PFL);
            FRpower.add(PFR);
            BLpower.add(PBL);
            BRpower.add(PBR);

            LoopTime.add(currenTime - prevTime);
            deltaX.add(currentx);
            deltaY.add(currenty);
            Angle.add(currentTheta);

            telemetry.addData("pinpointa", currentTheta);
            telemetry.addData("pinpointx", currentx);
            telemetry.addData("pinpointy", currenty);
            telemetry.addData("deltaY", deltay);
            telemetry.addData("deltaX", deltax);
            telemetry.addData("deltaA", deltaA);
//            telemetry.addData("DFL",DFL);
//            telemetry.addData("DFR",DFR);
//            telemetry.addData("DBL",DBL);
//            telemetry.addData("DBR",DBR);
            telemetry.update();

            prevDeltaAll = deltaAll;
            prevTime = currenTime;
        }

        File f = FileUtil.getfile();
        RobotLog.i("Writing file to " + f);
        try (FileOutputStream fos = new FileOutputStream(f);
             OutputStreamWriter writer = new OutputStreamWriter(fos, StandardCharsets.UTF_8)
        ) {
            writer.write("time,velx,vely,SFL,SFR,SBL,SBR,PFL,PFR,PBL,PBR,LoopTime,deltaX,deltaY,Angle\n");
            for (int i = 0; i < Time.size(); i++) {
                writer.write(String.format(
                        Locale.ROOT,
                        "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                        Time.get(i),
                        VelocityX.get(i),
                        VelocityY.get(i),
                        FLspeed.get(i),
                        FRspeed.get(i),
                        BLspeed.get(i),
                        BRspeed.get(i),
                        FLpower.get(i),
                        FRpower.get(i),
                        BLpower.get(i),
                        BRpower.get(i),
                        LoopTime.get(i),
                        deltaX.get(i),
                        deltaY.get(i),
                        Angle.get(i)

                ));
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public double speed2Power(double speed) {
        final double threshold = 0.2;

        if (Math.abs(speed) < 0.001) {
            return 0;
        }

        if (speed > 0) {
            return threshold + ((1 - threshold) * speed);
        }
        if (speed < 0) {
            return -threshold + ((1 - threshold) * speed);
        }

        throw new IllegalArgumentException();
    }

    public void spindexer0() {


        while (true) {
            boolean indexer3 = hardware.idxMag3.getState();
            boolean indexer2 = hardware.idxMag2.getState();
            hardware.indexer.setVelocity(250);


            if (!indexer2 && !indexer3) {
                hardware.indexer.setVelocity(0);
                break;
            }
        }
    }


    public void drive2Pose2(double[] xya) {
        ArrayList<Long> Time = new ArrayList<>();
        ArrayList<Double> VelocityX = new ArrayList<>();
        ArrayList<Double> VelocityY = new ArrayList<>();

        ArrayList<Double> FLspeed = new ArrayList<>();
        ArrayList<Double> BLspeed = new ArrayList<>();
        ArrayList<Double> FRspeed = new ArrayList<>();
        ArrayList<Double> BRspeed = new ArrayList<>();

        ArrayList<Double> FLpower = new ArrayList<>();
        ArrayList<Double> BLpower = new ArrayList<>();
        ArrayList<Double> FRpower = new ArrayList<>();
        ArrayList<Double> BRpower = new ArrayList<>();

        ArrayList<Double> LoopTime = new ArrayList<>();
        ArrayList<Double> deltaX = new ArrayList<>();
        ArrayList<Double> deltaY = new ArrayList<>();
        ArrayList<Double> Angle = new ArrayList<>();
//        hardware.PinPoint.setOffsets(3.4,1, DistanceUnit.INCH);
//        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

//        hardware.PinPoint.resetPosAndIMU();
//        hardware.PinPoint.recalibrateIMU();
//        hardware = new DumbledoreHardware(hardwareMap);

        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        double Fkp = 0;
        double Fkd = 0;
        double Fki = 0;

        double Skp = 0;
        double Skd = 0;
        double Ski = 0;

        double Wkp = 0;
        double Wkd = 0;
        double Wki = 0;

        double currenTime = runtime.time();
        double prevTime = currenTime;
        double prevDeltaAll = 0;


        double tgtx = xya[0];
        double tgty = xya[1];
        double tgta = xya[2];

        double sumF = 0;
        double sumS = 0;
        double sumW = 0;


        while (true) {
            currenTime = runtime.time();

            double Timeout = timeout.time();

            hardware.pinpoint.update();

            double yVelocity = hardware.pinpoint.getVelY(DistanceUnit.INCH);
            double xVelocity = hardware.pinpoint.getVelX(DistanceUnit.INCH);
            double angVelocity = hardware.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

            double speed = Math.sqrt((yVelocity * yVelocity) + (xVelocity * xVelocity));

            Pose2D currentPose = hardware.pinpoint.getPosition();

            double currentx = currentPose.getX(DistanceUnit.INCH);
            double currenty = currentPose.getY(DistanceUnit.INCH);
            double currentTheta = currentPose.getHeading(AngleUnit.RADIANS);

            double deltax = tgtx - currentx;
            double deltay = tgty - currenty;
            double deltaA = tgta - currentTheta;
            deltaA = deltaA % (2 * (Math.PI));
            if (deltaA > Math.PI) {
                deltaA -= 2 * Math.PI;
            } else if (deltaA < -Math.PI) {
                deltaA += 2* Math.PI;
            }

            if ((Math.abs(deltax) < 0.5 && Math.abs(deltay) < 0.5 && Math.abs(deltaA) < Math.PI / 24 && speed < 10) || Timeout > 1) {
                hardware.frontLeft.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backRight.setPower(0);
                break;
            }

            double R = 9.375;
            double F = Math.cos(currentTheta) * deltax + Math.sin(currentTheta) * deltay;
            double S = Math.sin(currentTheta) * deltax - Math.cos(currentTheta) * deltay;
            double W = R * deltaA;

            double vF = Math.cos(currentTheta) * xVelocity + Math.sin(currentTheta) * yVelocity; //velocity in the F direction
            double vS = Math.sin(currentTheta) * xVelocity - Math.cos(currentTheta) * yVelocity; //velocity in the S direction
            double vW = R * angVelocity;

            sumF += F; //the errors for the i term
            sumS += S;
            sumW += W;

            double PF = Fkp*F + Fki*sumF - Fkd*vF; //using velocity instead of (currentF-prevF)/deltaT because loop times varied a lot when we were recording them. idk if it'll make any difference
            double PS = Skp*S + Ski*sumS - Skd*vS;
            double PW = Wkp*W + Wki*sumW - Wkd*vW;

            double deltaAll = Math.sqrt((F * F) + (S * S) + (W * W));

            if (Math.abs(deltaAll - prevDeltaAll) > 0.5) {
                timeout.reset();
            }

            double PFL = PF + PS - PW;
            double PBL = PF - PS - PW;
            double PFR = PF - PS + PW;
            double PBR = PF + PS + PW;

            //rescale the four speeds if one is larger than abs(1)

            double tempMax1 = Math.max(Math.abs(PFL), Math.abs(PBL));
            double tempMax2 = Math.max(Math.abs(PFR), Math.abs(PBR));
            double scale = Math.max(tempMax1, tempMax2);


            if (scale>1) {
                PFL /= scale;
                PBL /= scale;
                PFR /= scale;
                PBR /= scale;
                sumF = 0; //if you're so far that you need to scale the powers down, don't start adding up errors for i
                sumS = 0;
                sumW = 0;
            }


            hardware.frontLeft.setPower(PFL);
            hardware.backLeft.setPower(PBL);
            hardware.frontRight.setPower(PFR);
            hardware.backRight.setPower(PBR);

            Time.add(System.nanoTime());
            VelocityY.add(yVelocity);
            VelocityX.add(xVelocity);
            FLspeed.add(PFL);
            FRspeed.add(PFR);
            BLspeed.add(PBL);
            BRspeed.add(PBR);

            FLpower.add(PFL);
            FRpower.add(PFR);
            BLpower.add(PBL);
            BRpower.add(PBR);

            LoopTime.add(currenTime - prevTime);
            deltaX.add(currentx);
            deltaY.add(currenty);
            Angle.add(currentTheta);

            telemetry.addData("pinpointa", currentTheta);
            telemetry.addData("pinpointx", currentx);
            telemetry.addData("pinpointy", currenty);
            telemetry.addData("deltaY", deltay);
            telemetry.addData("deltaX", deltax);
            telemetry.addData("deltaA", deltaA);
//            telemetry.addData("DFL",DFL);
//            telemetry.addData("DFR",DFR);
//            telemetry.addData("DBL",DBL);
//            telemetry.addData("DBR",DBR);
            telemetry.update();

            prevDeltaAll = deltaAll;
            prevTime = currenTime;
        }

        File f = FileUtil.getfile();
        RobotLog.i("Writing file to " + f);
        try (FileOutputStream fos = new FileOutputStream(f);
             OutputStreamWriter writer = new OutputStreamWriter(fos, StandardCharsets.UTF_8)
        ) {
            writer.write("time,velx,vely,SFL,SFR,SBL,SBR,PFL,PFR,PBL,PBR,LoopTime,deltaX,deltaY,Angle\n");
            for (int i = 0; i < Time.size(); i++) {
                writer.write(String.format(
                        Locale.ROOT,
                        "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                        Time.get(i),
                        VelocityX.get(i),
                        VelocityY.get(i),
                        FLspeed.get(i),
                        FRspeed.get(i),
                        BLspeed.get(i),
                        BRspeed.get(i),
                        FLpower.get(i),
                        FRpower.get(i),
                        BLpower.get(i),
                        BRpower.get(i),
                        LoopTime.get(i),
                        deltaX.get(i),
                        deltaY.get(i),
                        Angle.get(i)

                ));
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }


}
