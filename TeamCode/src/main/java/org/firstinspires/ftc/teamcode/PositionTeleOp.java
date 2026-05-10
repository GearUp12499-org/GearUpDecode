package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver;
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Locale;

@TeleOp
public class  PositionTeleOp extends LinearOpMode {

    final double INCHES_PER_METER = 39.37;
    final double COORD_FLIP = -1;

    final double r_turret = 6.5;   // turret radius
    final double t_offset = 0.5;   // offset from center

    final double x_l = 24;
    final double y_l = -24;
    Limelight3A limelight3A;

    CompBot2Hardware hardware;


    @Override
    public void runOpMode() throws InterruptedException {

        ArrayList<Double> distance = new ArrayList<>();
        ArrayList<Double> llX = new ArrayList<>();
        ArrayList<Double> llY = new ArrayList<>();
        ArrayList<Double> ll2ppX = new ArrayList<>();
        ArrayList<Double> ll2ppY = new ArrayList<>();
        ArrayList<Double> pinpointX = new ArrayList<>();
        ArrayList<Double> pinpointY = new ArrayList<>();
        ArrayList<Double> diffX = new ArrayList<>();
        ArrayList<Double> diffY = new ArrayList<>();


        hardware = new CompBot2Hardware(hardwareMap);
 
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.setPollRateHz(100);
        limelight3A.start();
        hardware.pinpoint.setPosition(
                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0)
        );

        boolean wasY = false;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                File f = FileUtil.getfile();
                RobotLog.i("Writing file to " + f);
                try (FileOutputStream fos = new FileOutputStream(f);
                     OutputStreamWriter writer = new OutputStreamWriter(fos, StandardCharsets.UTF_8)
                ) {
                    writer.write("distance,llx,lly,ll2ppx,ll2ppy,pinpointx,pinpointy,diifx,diffy\n");
                    for (int i = 0; i < distance.size(); i++) {
                        writer.write(String.format(
                                Locale.ROOT,
                                "%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                                distance.get(i),
                                llX.get(i),
                                llY.get(i),
                                ll2ppX.get(i),
                                ll2ppY.get(i),
                                pinpointX.get(i),
                                pinpointY.get(i),
                                diffX.get(i),
                                diffY.get(i)
                        ));
                    }
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }

            hardware.pinpoint.update();
            Pose2D currentPose = hardware.pinpoint.getPosition();

            double ppX = currentPose.getX(DistanceUnit.INCH);
            double ppY = currentPose.getY(DistanceUnit.INCH);
            double ppTheta = currentPose.getHeading(AngleUnit.RADIANS);

            LLResult result = limelight3A.getLatestResult();

            if (result != null && result.isValid()) {

//                limelight3A.updateRobotOrientation(currentPose.getHeading(AngleUnit.DEGREES) + 90 + hardware.turretEncoder.getCurrentPosition()/159.5);

                limelight3A.updateRobotOrientation((currentPose.getHeading(AngleUnit.DEGREES)+(-hardware.turretEncoder.getCurrentPosition() / 159.5)));
                Pose3D botpose = result.getBotpose_MT2();
//                Pose3D botpose = result.getBotpose();

                if (botpose != null) {

                    double limelightX = botpose.getPosition().x * INCHES_PER_METER * COORD_FLIP;
                    double limelightY = botpose.getPosition().y * INCHES_PER_METER * COORD_FLIP;

                    //double theta_r = botpose.getOrientation().getYaw();
                    double theta_r = hardware.pinpoint.getHeading(AngleUnit.RADIANS);

                    //Limelight angle to tag (not sure if this is the right conversion)
//                    double theta_l = -turret.currentPosition() / TICKS_PER_DEG) * (PI / 180);
                    double theta_l = (-hardware.turretEncoder.getCurrentPosition()/159.5) * (Math.PI/180);

                    double d = Math.sqrt(
                            r_turret*r_turret +
                                    t_offset*t_offset -
                                    2*r_turret*t_offset*Math.cos(Math.PI - theta_l)
                    );

                    double x = Math.asin(
                            (Math.sin(Math.PI - theta_l) * r_turret) / d
                    );

                    double f = d * Math.cos(x);
                    double s = d * Math.sin(x);

                    double x_offset = f * Math.cos(theta_r) - s * Math.sin(theta_r);
                    double y_offset = f * Math.sin(theta_r) + s * Math.cos(theta_r);

                    double x_robot = limelightX + x_offset;
                    double y_robot = limelightY + y_offset;

                    YawPitchRollAngles ypr = hardware.imu.getRobotYawPitchRollAngles();

                    telemetry.addData("Limelight X", limelightX);
                    telemetry.addData("Limelight Y", limelightY);
                    telemetry.addData("Field X", x_robot);
                    telemetry.addData("Field Y", y_robot);
                    telemetry.addData("pinpoint X", ppX);
                    telemetry.addData("pinpoint Y", ppY);
                    telemetry.addData("pinpoint Heading", ppTheta);
                    telemetry.addData("yaw", ypr.getYaw());
                    telemetry.addData("pitch", ypr.getPitch());
                    telemetry.addData("roll", ypr.getRoll());
                    telemetry.addData("pinpoint yaw", currentPose.getHeading(AngleUnit.DEGREES));



                    if (gamepad1.y && !wasY) {
                        distance.add((Math.hypot(currentPose.getX(DistanceUnit.INCH)-58,currentPose.getY(DistanceUnit.INCH)+56)));
                        llX.add(limelightX);
                        llY.add(limelightY);
                        ll2ppX.add(x_robot);
                        ll2ppY.add(y_robot);
                        pinpointX.add(ppX);
                        pinpointY.add(ppY);
                        diffX.add(x_robot-ppX);
                        diffY.add(y_robot-ppY);
                    }


                    wasY = gamepad1.y;
                }
            }

            telemetry.update();
        }


}
    }

