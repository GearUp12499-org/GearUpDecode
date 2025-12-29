package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver;
import org.firstinspires.ftc.teamcode.systems.REmover;
import org.firstinspires.ftc.teamcode.tasks.PinpointUpdater;
import org.firstinspires.ftc.teamcode.tasks.SentinelTask;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;

import io.github.gearup12499.taskshark.FastScheduler;
import io.github.gearup12499.taskshark.Scheduler;
import io.github.gearup12499.taskshark.prefabs.OneShot;
import io.github.gearup12499.taskshark.prefabs.VirtualGroup;
import io.github.gearup12499.taskshark_android.TaskSharkAndroid;

@TeleOp
public class AutoBalance extends LinearOpMode {
    private ElapsedTime runtime;

    CompBotHardware hardware;

    FastScheduler scheduler;

    SentinelTask startFlag;

    double FLpower = 0;
    double BLpower = 0;
    double FRpower = 0;
    double BRpower = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        TaskSharkAndroid.setup();

        hardware = new CompBotHardware(hardwareMap);
        scheduler = new FastScheduler();
        startFlag = scheduler.add(new SentinelTask());

        hardware.pinpoint.recalibrateIMU();
        hardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-0,0, AngleUnit.RADIANS, 0));
        hardware.pinpoint.setOffsets(-3.9, -3.875 + 0.05, DistanceUnit.INCH);
        hardware.pinpoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        hardware.pinpoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.REVERSED, GoBildaPinpoint2Driver.EncoderDirection.FORWARD);

        while (opModeIsActive()) {
            scheduler.tick();

            scheduler.add(new PinpointUpdater(hardware.pinpoint));

           if (gamepad1.a){
               balance();
           }

           if (gamepad1.b){
               hardware.frontLeft.setPower(FLpower);
               hardware.frontRight.setPower(FRpower);
               hardware.backLeft.setPower(BLpower);
               hardware.backRight.setPower(BRpower);
           }

            else if (gamepad1.x){
               hardware.frontLeft.setPower(-FLpower);
               hardware.frontRight.setPower(FRpower);
               hardware.backLeft.setPower(BLpower);
               hardware.backRight.setPower(-BRpower);
            }

            else if (gamepad1.y){
               hardware.frontLeft.setPower(-FLpower);
               hardware.frontRight.setPower(FRpower);
               hardware.backLeft.setPower(-BLpower);
               hardware.backRight.setPower(BRpower);
            }
            else{
                hardware.frontLeft.setPower(0);
                hardware.frontRight.setPower(0);
                hardware.backLeft.setPower(0);
                hardware.backRight.setPower(0);
            }


        }
    }

    public void balance(){



        for (double start = 0.75; start <= 0.76; start += 0.25) {
            FLpower = start;
            FRpower = start;
            BLpower = start;
            BRpower = start;
            double delta = 0.1;

            for (int i = 1; i <= 7 ; i++) {



                int BestMult = -1000;
                double BestOffset = 10000;

//              forward
                for (int mult = -1; mult <= 1; mult++){
                    drive2Pose2(new REmover.RobotPose(0,0,0),1);

                    double TempFLpower = FLpower + mult * delta;
                    double TempFRpower = FRpower - mult * delta;
                    double TempBLpower = BLpower + mult * delta;
                    double TempBRpower = BRpower - mult * delta;

                    hardware.frontLeft.setPower(TempFLpower);
                    hardware.backLeft.setPower(TempBLpower);
                    hardware.frontRight.setPower(TempFRpower);
                    hardware.backRight.setPower(TempBRpower);



                    while(hardware.pinpoint.getPosX(DistanceUnit.INCH) < 48 && Math.abs(hardware.pinpoint.getPosY(DistanceUnit.INCH)) < 24 && hardware.pinpoint.getPosX(DistanceUnit.INCH) > -1){
                        hardware.pinpoint.update();
                    }

                    hardware.frontLeft.setPower(0);
                    hardware.backLeft.setPower(0);
                    hardware.frontRight.setPower(0);
                    hardware.backRight.setPower(0);

                    double currentError = Math.abs(hardware.pinpoint.getPosY(DistanceUnit.INCH));

                    if(hardware.pinpoint.getPosX(DistanceUnit.INCH) < -1){
                        currentError = 100000000;
                    }


                    if(currentError < BestOffset){
                        BestOffset = currentError;
                        BestMult = mult;
                    }

                    Log.i("xPos", String.valueOf(hardware.pinpoint.getPosX(DistanceUnit.INCH)));
                    Log.i("yPos", String.valueOf(hardware.pinpoint.getPosY(DistanceUnit.INCH)));
                    Log.i("Heading", String.valueOf(hardware.pinpoint.getHeading(AngleUnit.RADIANS)));


                }

                FLpower = FLpower + BestMult * delta;
                FRpower = FRpower - BestMult * delta;
                BLpower = BLpower + BestMult * delta;
                BRpower = BRpower - BestMult * delta;

                telemetry.addData("FL",FLpower);
                telemetry.addData("FR",FRpower);
                telemetry.addData("BL",BLpower);
                telemetry.addData("BR",BRpower);

                telemetry.update();

                Log.i("LeftPower", String.valueOf(FLpower));
                Log.i("RightPower", String.valueOf(FRpower));

                BestMult = -1000;
                BestOffset = 100000;

//                strafe
                for (int mult = -1; mult <= 1; mult++) {
                    drive2Pose2(new REmover.RobotPose(0,0,Math.PI/2),1);

                    double TempFLpower =  (FLpower + mult * delta);
                    double TempFRpower = -(FRpower + mult * delta);
                    double TempBLpower = -(BLpower - mult * delta);
                    double TempBRpower =  (BRpower - mult * delta);

                    hardware.frontLeft.setPower(TempFLpower);
                    hardware.backLeft.setPower(TempBLpower);
                    hardware.frontRight.setPower(TempFRpower);
                    hardware.backRight.setPower(TempBRpower);



                    while(hardware.pinpoint.getPosX(DistanceUnit.INCH) < 48 && Math.abs(hardware.pinpoint.getPosY(DistanceUnit.INCH)) < 24 && hardware.pinpoint.getPosX(DistanceUnit.INCH) > -1){
                        hardware.pinpoint.update();
                    }

                    double currentError = Math.abs(hardware.pinpoint.getPosY(DistanceUnit.INCH));

                    if(hardware.pinpoint.getPosX(DistanceUnit.INCH) < -1){
                        currentError = 100000000;
                    }

                    if(currentError < BestOffset){
                        BestOffset = currentError;
                        BestMult = mult;
                    }

                    hardware.frontLeft.setPower(0);
                    hardware.backLeft.setPower(0);
                    hardware.frontRight.setPower(0);
                    hardware.backRight.setPower(0);



                    Log.i("xPos", String.valueOf(hardware.pinpoint.getPosX(DistanceUnit.INCH)));
                    Log.i("yPos", String.valueOf(hardware.pinpoint.getPosY(DistanceUnit.INCH)));
                    Log.i("Heading", String.valueOf(hardware.pinpoint.getHeading(AngleUnit.RADIANS)));
                }

                FLpower =  (FLpower + BestMult * delta);
                FRpower =  (FRpower + BestMult * delta);
                BLpower =  (BLpower - BestMult * delta);
                BRpower =  (BRpower - BestMult * delta);


                telemetry.addData("StrafeFL",FLpower);
                telemetry.addData("StrafeFR",FRpower);
                telemetry.addData("StrafeBL",BLpower);
                telemetry.addData("StrafeBR",BRpower);

                telemetry.update();

                BestMult = -1000;
                BestOffset = 100000;

                for (int mult = -1; mult <= 1; mult++) {
                    drive2Pose2(new REmover.RobotPose(0,0,0),1);


                    double TempFLpower =  -(FLpower + mult * delta);
                    double TempFRpower =   (FRpower - mult * delta);
                    double TempBLpower =  -(BLpower - mult * delta);
                    double TempBRpower =   (BRpower + mult * delta);

                    double StartHeading = hardware.pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);

                    hardware.frontLeft.setPower(TempFLpower);
                    hardware.backLeft.setPower(TempBLpower);
                    hardware.frontRight.setPower(TempFRpower);
                    hardware.backRight.setPower(TempBRpower);

                    while(hardware.pinpoint.getHeading(AngleUnit.RADIANS.getUnnormalized()) - StartHeading < Math.PI){
                        hardware.pinpoint.update();
                    }

                    double currentError = Math.abs(Math.hypot(hardware.pinpoint.getPosY(DistanceUnit.INCH), hardware.pinpoint.getPosX(DistanceUnit.INCH)));

                    if(currentError < BestOffset){
                        BestOffset = currentError;
                        BestMult = mult;
                    }

                    hardware.frontLeft.setPower(0);
                    hardware.backLeft.setPower(0);
                    hardware.frontRight.setPower(0);
                    hardware.backRight.setPower(0);



                    Log.i("xPos", String.valueOf(hardware.pinpoint.getPosX(DistanceUnit.INCH)));
                    Log.i("yPos", String.valueOf(hardware.pinpoint.getPosY(DistanceUnit.INCH)));
                    Log.i("Heading", String.valueOf(hardware.pinpoint.getHeading(AngleUnit.RADIANS)));
                }

                FLpower =  (FLpower + BestMult * delta);
                FRpower =  (FRpower - BestMult * delta);
                BLpower =  (BLpower - BestMult * delta);
                BRpower =  (BRpower + BestMult * delta);

                telemetry.addData("FL",FLpower);
                telemetry.addData("FR",FRpower);
                telemetry.addData("BL",BLpower);
                telemetry.addData("BR",BRpower);

                telemetry.update();

                Log.i("BLPower", String.valueOf(BLpower));
                Log.i("BRPower", String.valueOf(BRpower));
                Log.i("FLPower", String.valueOf(FLpower));
                Log.i("FRPower", String.valueOf(FRpower));
                delta *= .7;
            }
            Log.i("QQQBLPower", String.valueOf(BLpower));
            Log.i("QQQBRPower", String.valueOf(BRpower));
            Log.i("QQQFLPower", String.valueOf(FLpower));
            Log.i("QQQFRPower", String.valueOf(FRpower));
        }
    }
    public void drive2Pose2(REmover.RobotPose xya, double maxPower) {
//        ArrayList<Long> Time = new ArrayList<>();
//        ArrayList<Double> VelocityX = new ArrayList<>();
//        ArrayList<Double> VelocityY = new ArrayList<>();
//
//        ArrayList<Double> FLspeed = new ArrayList<>();
//        ArrayList<Double> BLspeed = new ArrayList<>();
//        ArrayList<Double> FRspeed = new ArrayList<>();
//        ArrayList<Double> BRspeed = new ArrayList<>();
//
//        ArrayList<Double> FLpower = new ArrayList<>();
//        ArrayList<Double> BLpower = new ArrayList<>();
//        ArrayList<Double> FRpower = new ArrayList<>();
//        ArrayList<Double> BRpower = new ArrayList<>();
//
//        ArrayList<Double> LoopTime = new ArrayList<>();
//        ArrayList<Double> deltaX = new ArrayList<>();
//        ArrayList<Double> deltaY = new ArrayList<>();
//        ArrayList<Double> Angle = new ArrayList<>();
//        hardware.PinPoint.setOffsets(3.4,1, DistanceUnit.INCH);
//        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

//        hardware.PinPoint.resetPosAndIMU();
//        hardware.PinPoint.recalibrateIMU();
//        hardware = new DumbledoreHardware(hardwareMap);

        if (maxPower > 1) {
            maxPower = 1;
        }

        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        double Fkp = REmover.FKP;
        double Fkd = REmover.FKD;
        double Fki = REmover.FKI;

        double Skp = REmover.SKP;
        double Skd = REmover.SKD;
        double Ski = REmover.SKI;

        double Wkp = REmover.WKP;
        double Wkd = REmover.WKD;
        double Wki = REmover.WKI;

        // milliseconds
        double currenTime = runtime.time();
        double prevTime = currenTime;
        double prevDeltaAll = 0;


        double tgtx = xya.x;
        double tgty = xya.y;
        double tgta = xya.a;

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
//if x pos, y pos, and angle are close enough, and if x vel, y vel, and angle vel are slow enough, or when you time out (stuck for too long), exit the loop
            if ((abs(deltax) < 0.5 && abs(deltay) < 0.5 && abs(deltaA) < Math.PI / 48 && speed < 10) && abs(angVelocity) < Math.PI/4|| Timeout > 1) {
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

            double deltaTime = currenTime - prevTime;

            double vF = Math.cos(currentTheta) * xVelocity + Math.sin(currentTheta) * yVelocity; //velocity in the F direction
            double vS = Math.sin(currentTheta) * xVelocity - Math.cos(currentTheta) * yVelocity; //velocity in the S direction
            double vW = R * angVelocity;

            if (Math.abs(F) > 1) {
                sumF = 0;
            } else {
                sumF += F*deltaTime;
            }

            if (Math.abs(S) > 1) {
                sumS = 0;
            } else {
                sumS += S*deltaTime;
            }

            if (Math.abs(W) < 3) {
                sumW = 0;
            } else {
                sumW += W*deltaTime;
            }

            double PF = Fkp*F + Fki*sumF - Fkd*vF; //using velocity instead of (currentF-prevF)/deltaT because loop times varied a lot when we were recording them. idk if it'll make any difference
            double PS = Skp*S + Ski*sumS - Skd*vS;
            double PW = Wkp*W + Wki*sumW - Wkd*vW;

            double deltaAll = Math.sqrt((F * F) + (S * S) + (W * W));

            if (abs(deltaAll - prevDeltaAll) > 0.5) {
                timeout.reset();
            }

            double PFL = PF + PS - PW;
            double PBL = PF - PS - PW;
            double PFR = PF - PS + PW;
            double PBR = PF + PS + PW;

            //rescale the four speeds if one is larger than abs(1)

//            double tempMax1 = Math.max(Math.abs(PFL), Math.abs(PBL));
//            double tempMax2 = Math.max(Math.abs(PFR), Math.abs(PBR));
//            double scale = Math.max(tempMax1, tempMax2);
//
//
//            if (scale>1) {
//                PFL /= (scale/maxSpeed);
//                PBL /= scale;
//                PFR /= scale;
//                PBR /= scale;
//            }

            double tempMax1 = Math.max(abs(PFL), abs(PBL));
            double tempMax2 = Math.max(abs(PFR), abs(PBR));
            double greatestPower = Math.max(tempMax1, tempMax2);

            if (greatestPower>maxPower) {
                double scale = greatestPower/maxPower;
                PFL /= scale;
                PBL /= scale;
                PFR /= scale;
                PBR /= scale;
            }

//            PFL = speed2Power(PFL);
//            PBL = speed2Power(PBL);
//            PFR = speed2Power(PFR);
//            PBR = speed2Power(PBR);


            hardware.frontLeft.setPower(PFL);
            hardware.backLeft.setPower(PBL);
            hardware.frontRight.setPower(PFR);
            hardware.backRight.setPower(PBR);

//            Time.add(System.nanoTime());
//            VelocityY.add(yVelocity);
//            VelocityX.add(xVelocity);
//            FLspeed.add(PFL);
//            FRspeed.add(PFR);
//            BLspeed.add(PBL);
//            BRspeed.add(PBR);
//
//            FLpower.add(PFL);
//            FRpower.add(PFR);
//            BLpower.add(PBL);
//            BRpower.add(PBR);
//
//            LoopTime.add(currenTime - prevTime);
//            deltaX.add(currentx);
//            deltaY.add(currenty);
//            Angle.add(currentTheta);

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

//        File f = FileUtil.getfile();
//        RobotLog.i("Writing file to " + f);
//        try (FileOutputStream fos = new FileOutputStream(f);
//             OutputStreamWriter writer = new OutputStreamWriter(fos, StandardCharsets.UTF_8)
//        ) {
//            writer.write("time,velx,vely,SFL,SFR,SBL,SBR,PFL,PFR,PBL,PBR,LoopTime,deltaX,deltaY,Angle\n");
//            for (int i = 0; i < Time.size(); i++) {
//                writer.write(String.format(
//                        Locale.ROOT,
//                        "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
//                        Time.get(i),
//                        VelocityX.get(i),
//                        VelocityY.get(i),
//                        FLspeed.get(i),
//                        FRspeed.get(i),
//                        BLspeed.get(i),
//                        BRspeed.get(i),
//                        FLpower.get(i),
//                        FRpower.get(i),
//                        BLpower.get(i),
//                        BRpower.get(i),
//                        LoopTime.get(i),
//                        deltaX.get(i),
//                        deltaY.get(i),
//                        Angle.get(i)
//
//                ));
    }
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }

}
