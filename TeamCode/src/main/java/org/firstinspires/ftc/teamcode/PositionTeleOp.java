package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver;
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

@TeleOp
public class PositionTeleOp extends LinearOpMode {

    final double INCHES_PER_METER = 39.37;
    final double COORD_FLIP = -1;

    final double r_turret = 6;   // turret radius
    final double t_offset = 1;   // offset from center

    final double x_l = 24;
    final double y_l = -24;
    GoBildaPinpoint2Driver pinpoint;
    Limelight3A limelight3A;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2);
        limelight3A.setPollRateHz(100);
        limelight3A.start();
        pinpoint.setPosition(
                new Pose2D(DistanceUnit.INCH, -63, -16, AngleUnit.RADIANS, -Math.PI)
        );
        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            double ppX = currentPose.getX(DistanceUnit.INCH);
            double ppY = currentPose.getY(DistanceUnit.INCH);
            double ppTheta = currentPose.getHeading(AngleUnit.RADIANS);

            LLResult result = limelight3A.getLatestResult();

            if (result != null && result.isValid()) {

                Pose3D botpose = result.getBotpose();

                if (botpose != null) {

                    double limelightX = botpose.getPosition().x * INCHES_PER_METER * COORD_FLIP;
                    double limelightY = botpose.getPosition().y * INCHES_PER_METER * COORD_FLIP;

                    double theta_r = botpose.getOrientation().getYaw();

                    //Limelight angle to tag (not sure if this is the right conversion)
                    double theta_l = Math.PI/2;


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

                    double x_robot = x_l + x_offset;
                    double y_robot = y_l + y_offset;


                    telemetry.addData("Limelight X", limelightX);
                    telemetry.addData("Limelight Y", limelightY);
                    telemetry.addData("Field X", x_robot);
                    telemetry.addData("Field Y", y_robot);
                    telemetry.addData("pinpoint X", ppX);
                    telemetry.addData("pinpoint Y", ppY);
                    telemetry.addData("pinpoint Heading", ppTheta);
                }
            }

            telemetry.update();
        }
    }
}
