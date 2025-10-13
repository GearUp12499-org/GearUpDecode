package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.DumbledoreHardware;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Locale;

@TeleOp
public class DumbledoreTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    DumbledoreHardware hardware;


    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new DumbledoreHardware(hardwareMap);

        hardware.PinPoint.setOffsets(3.38,-1.10, DistanceUnit.INCH);
//        hardware.PinPoint.setOffsets(0,0,DistanceUnit.INCH);


        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        hardware.PinPoint.resetPosAndIMU();
        hardware.PinPoint.recalibrateIMU();

//        sleep(5000);
//
        hardware.PinPoint.setOffsets(3.4,1, DistanceUnit.INCH);

        telemetry.addData("pose",hardware.PinPoint.getPosition());
        telemetry.update();


        waitForStart();

        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,-16, AngleUnit.RADIANS,0));


        while (opModeIsActive()){
            hardware.PinPoint.update();

            Pose2D currentPose = hardware.PinPoint.getPosition();

            telemetry.addData("pinpointa",currentPose.getHeading(AngleUnit.RADIANS));
            telemetry.addData("pinpointx",currentPose.getX(DistanceUnit.INCH));
            telemetry.addData("pinpointy",currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("pinpoint",hardware.PinPoint.getDeviceStatus());
//            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
//            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
//            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("imu angle (DEGREES)", hardware.PinPoint.getHeading(AngleUnit.DEGREES));


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            hardware.frontLeft.setPower(frontLeftPower);
            hardware.backLeft.setPower(backLeftPower);
            hardware.frontRight.setPower(frontRightPower);
            hardware.backRight.setPower(backRightPower);

            if(gamepad1.y){
                drive2Pose(0,0,0);

//                drive2Pose(78,0,-Math.PI/4 );
//                sleep(2000);
//                drive2Pose(78,-18,-Math.PI/2);
//                sleep(2000);
//                drive2Pose(24,-16.5,-Math.PI/2);
            }

            if (gamepad1.a){
                drive2Pose(72,0,0);
//                hardware.frontLeft.setPower(-1);
//                hardware.backLeft.setPower(-1);
//                hardware.frontRight.setPower(-1);
//                hardware.backRight.setPower(-1);
            }
            if (gamepad1.b){
                hardware.frontLeft.setPower(-1);
                hardware.backLeft.setPower(1);
                hardware.frontRight.setPower(1);
                hardware.backRight.setPower(-1);
            }
            if (gamepad1.x){
                hardware.frontLeft.setPower(1);
                hardware.backLeft.setPower(-1);
                hardware.frontRight.setPower(-1);
                hardware.backRight.setPower(1);
            }

            if (gamepad1.right_bumper) {
                while (hardware.PinPoint.getPosX(DistanceUnit.INCH) >= -48) {
                    hardware.frontLeft.setPower(-0.3);
                    hardware.backLeft.setPower(-0.3);
                    hardware.frontRight.setPower(-0.3);
                    hardware.backRight.setPower(-0.3);
                    hardware.PinPoint.update();
                }
                hardware.frontLeft.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backRight.setPower(0);
            }
            if (gamepad1.left_bumper) {
                while (hardware.PinPoint.getPosX(DistanceUnit.INCH) <= 0) {
                    hardware.frontLeft.setPower(0.3);
                    hardware.backLeft.setPower(0.3);
                    hardware.frontRight.setPower(0.3);
                    hardware.backRight.setPower(0.3);
                    hardware.PinPoint.update();
                }
                hardware.frontLeft.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backRight.setPower(0);
            }
            telemetry.update();

            telemetry.update();
        }
    }
    public void drive2Pose (double targetx, double targety, double targeta){


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
//        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0)); //Do we want to do this again?
//        hardware.PinPoint.setOffsets(3.4,1, DistanceUnit.INCH);
//        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

//        hardware.PinPoint.resetPosAndIMU();
//        hardware.PinPoint.recalibrateIMU();
//        hardware = new DumbledoreHardware(hardwareMap);

        double kp = 0.2;
        double kd = 43.75;

        double currenTime = runtime.time();
        double prevTime = runtime.time();
        double deltaTime = 0;
        double prevDeltaAll = 0;



        while(true) {
            currenTime = runtime.time();

            hardware.PinPoint.update();

            double yVelocity= hardware.PinPoint.getVelY(DistanceUnit.INCH);
            double xVelocity = hardware.PinPoint.getVelX(DistanceUnit.INCH);

            double speed = Math.sqrt((yVelocity*yVelocity)+(xVelocity*xVelocity));

            Pose2D currentPose = hardware.PinPoint.getPosition();

            double currentx = currentPose.getX(DistanceUnit.INCH);
            double currenty = currentPose.getY(DistanceUnit.INCH);
            double currentTheta = currentPose.getHeading(AngleUnit.RADIANS);

            double deltax = targetx - currentx;
            double deltay = targety - currenty;
            double deltaA = targeta - currentTheta;
            deltaA = deltaA % (2*(Math.PI));
            if (deltaA > Math.PI) {
                deltaA -= 2 * Math.PI;
            }

            if(Math.abs(deltax)< 0.5 && Math.abs(deltay) < 0.5 && Math.abs(deltaA) < Math.PI/24 && speed < 10){
                hardware.frontLeft.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backRight.setPower(0);
                break;
            }

            double R = 9.375 ;
            double F = Math.cos(currentTheta) * deltax + Math.sin(currentTheta) * deltay;
            double S = Math.sin(currentTheta) * deltax - Math.cos(currentTheta) * deltay;
            double W = R * deltaA;
            double deltaAll = Math.sqrt((F*F) + (S*S) + (W*W));

            double DFL = F + S - W;
            double DBL = F - S - W;
            double DFR = F - S + W;
            double DBR = F + S + W;

            //rescale the four speeds so the largest is +/- 1
            double tempMax1 = Math.max(Math.abs(DFL),Math.abs(DBL));
            double tempMax2 =  Math.max(Math.abs(DFR),Math.abs(DBR));
            double scale = Math.max(tempMax1,tempMax2);

            if (scale<0.01){
                scale = 0.01;
            }

//            double scale = Math.max(Math.abs(F) + Math.abs(S) + Math.abs(W), kp*deltaAll);

            DFL /= scale;
            DBL /= scale;
            DFR /= scale;
            DBR /= scale;

            //PID computation

            deltaTime = Math.max(currenTime - prevTime,0.001);//making sure we don't divide by 0

            double pid = kp*deltaAll + kd*(deltaAll-prevDeltaAll)/deltaTime;

            prevDeltaAll = deltaAll;
            prevTime = currenTime;

            //if pid<1, rescale so fastest speed is +/- pid
            if (Math.abs(pid)<1){
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


            telemetry.addData("pinpointa",currentTheta);
            telemetry.addData("pinpointx",currentx);
            telemetry.addData("pinpointy",currenty);
            telemetry.addData("deltaY", deltay);
            telemetry.addData("deltaX", deltax);
            telemetry.addData("deltaA", deltaA);
//            telemetry.addData("DFL",DFL);
//            telemetry.addData("DFR",DFR);
//            telemetry.addData("DBL",DBL);
//            telemetry.addData("DBR",DBR);
            telemetry.update();


        }

        File f = FileUtil.getfile();
        RobotLog.i("Writing file to " + f);
        try (FileOutputStream fos = new FileOutputStream(f);
             OutputStreamWriter writer = new OutputStreamWriter(fos, StandardCharsets.UTF_8)
        ){
            writer.write("time,velx,vely,SFL,SFR,SBL,SBR,PFL,PFR,PBL,PBR\n");
            for(int i = 0; i<Time.size(); i++){
                writer.write(String.format(
                        Locale.ROOT,
                        "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
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
                        BRpower.get(i)

                ));
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    public double speed2Power(double speed){
        final double threshold = 0.2;

        if(Math.abs(speed)<0.001){
            return 0;
        }

        if(speed > 0){
            return threshold+((1-threshold)*speed);
        }
        if(speed <0){
            return -threshold+((1-threshold)*speed);
        }

        throw new IllegalArgumentException();
    }
}
