package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.DumbledoreHardware;

@TeleOp
public class DumbledoreTeleOp extends LinearOpMode {
    DumbledoreHardware hardware;


    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new DumbledoreHardware(hardwareMap);

        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0));
        hardware.PinPoint.setOffsets(3.4,1, DistanceUnit.INCH);
        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        hardware.PinPoint.resetPosAndIMU();
        hardware.PinPoint.recalibrateIMU();


        waitForStart();

        while (opModeIsActive()){
            hardware.PinPoint.update();

            Pose2D currentPose = hardware.PinPoint.getPosition();

            telemetry.addData("pinpointa",currentPose.getHeading(AngleUnit.RADIANS));
            telemetry.addData("pinpointx",currentPose.getX(DistanceUnit.INCH));
            telemetry.addData("pinpointy",currentPose.getY(DistanceUnit.INCH));
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
                drive2Pose(-24,0,Math.PI/2 );
            }

            if (gamepad1.a){
                hardware.frontLeft.setPower(-1);
                hardware.backLeft.setPower(-1);
                hardware.frontRight.setPower(-1);
                hardware.backRight.setPower(-1);
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

        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0)); //Do we want to do this again?
//        hardware.PinPoint.setOffsets(3.4,1, DistanceUnit.INCH);
//        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

//        hardware.PinPoint.resetPosAndIMU();
//        hardware.PinPoint.recalibrateIMU();
//        hardware = new DumbledoreHardware(hardwareMap);
        while(true) {
            hardware.PinPoint.update();

            Pose2D currentPose = hardware.PinPoint.getPosition();

            double currentx = currentPose.getX(DistanceUnit.INCH);
            double currenty = currentPose.getY(DistanceUnit.INCH);
            double currentTheta = currentPose.getHeading(AngleUnit.RADIANS);

            double kp = 0.2;

            double deltax = targetx - currentx;
            double deltay = targety - currenty;
            double deltaA = targeta - currentTheta;
            deltaA = deltaA % (2*(Math.PI));
            if (deltaA > Math.PI) {
                deltaA -= 2 * Math.PI;
            }

            if(Math.abs(deltax)< 0.5 && Math.abs(deltay) < 0.5 && Math.abs(deltaA) < Math.PI/24){
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

            double scale = Math.max(Math.abs(F) + Math.abs(S) + Math.abs(W), kp*deltaAll);

            hardware.frontLeft.setPower(DFL / (scale*2));
            hardware.backLeft.setPower(DBL / (scale*2));
            hardware.frontRight.setPower(DFR / (scale*2));
            hardware.backRight.setPower(DBR / (scale*2));

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


    }
}
