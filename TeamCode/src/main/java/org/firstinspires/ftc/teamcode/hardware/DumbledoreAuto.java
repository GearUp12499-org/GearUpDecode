package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FileUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Locale;

@Autonomous
public class DumbledoreAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    DumbledoreHardware hardware;

    @Override
    public void runOpMode(){
        hardware = new DumbledoreHardware(hardwareMap);

        hardware.PinPoint.setOffsets(3.38,-1.10, DistanceUnit.INCH);

        hardware.PinPoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        hardware.PinPoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.FORWARD, GoBildaPinpoint2Driver.EncoderDirection.FORWARD);


        hardware.PinPoint.resetPosAndIMU();
        hardware.PinPoint.recalibrateIMU();


        hardware.PinPoint.setOffsets(3.4,1, DistanceUnit.INCH);

        telemetry.addData("pose",hardware.PinPoint.getPosition());
        telemetry.update();


        waitForStart();

        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,-16, AngleUnit.RADIANS,0));

        drive2Pose(hardware.shootPos);


    }
    public void drive2Pose (double[] xya){


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

        double tgtx = xya[0];
        double tgty = xya[1];
        double tgta = xya[2];

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



            double deltax = tgtx - currentx;
            double deltay = tgty - currenty;
            double deltaA = tgta - currentTheta;
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
