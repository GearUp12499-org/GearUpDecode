package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver;

@TeleOp
public class CompBotTeleOp extends LinearOpMode {

    CompBotHardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new CompBotHardware(hardwareMap);

        hardware.PinPoint.setOffsets(3.9,3.875, DistanceUnit.INCH);
        hardware.PinPoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        hardware.PinPoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.FORWARD, GoBildaPinpoint2Driver.EncoderDirection.REVERSED);
        hardware.PinPoint.resetPosAndIMU();
        hardware.PinPoint.recalibrateIMU();

        telemetry.addData("pose",hardware.PinPoint.getPosition());
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            hardware.PinPoint.update();

            Pose2D currentPose = hardware.PinPoint.getPosition();

            telemetry.addData("pinpointa",currentPose.getHeading(AngleUnit.RADIANS));
            telemetry.addData("pinpointx",currentPose.getX(DistanceUnit.INCH));
            telemetry.addData("pinpointy",currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("pinpoint",hardware.PinPoint.getDeviceStatus());

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FrontLeftPower = (y+x+rx)/denominator;
            double FrontRightPower = (y-x-rx)/denominator;
            double BackRightPower = (y+x-rx)/denominator;
            double BackLeftPower = (y-x+rx)/denominator;

            hardware.frontLeft.setPower(FrontLeftPower);
            hardware.frontRight.setPower(FrontRightPower);
            hardware.backRight.setPower(BackRightPower);
            hardware.backLeft.setPower(BackLeftPower);

            telemetry.update();

        }
    }
}
