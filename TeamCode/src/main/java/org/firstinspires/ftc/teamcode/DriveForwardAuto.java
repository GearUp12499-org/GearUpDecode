package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.DumbledoreHardware;

import io.github.gearup12499.taskshark.FastScheduler;
import io.github.gearup12499.taskshark.prefabs.OneShot;
import io.github.gearup12499.taskshark.prefabs.Wait;
import io.github.gearup12499.taskshark.prefabs.WaitUntil;
import io.github.gearup12499.taskshark_android.TaskSharkAndroid;

@Autonomous
public class DriveForwardAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        TaskSharkAndroid.setup();
        DumbledoreHardware hardware = new DumbledoreHardware(hardwareMap);
        FastScheduler scheduler = new FastScheduler();

//        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
//        hardware.PinPoint.setOffsets(96, 24, DistanceUnit.MM);
//        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//
//
//        hardware.PinPoint.resetPosAndIMU();
//        hardware.PinPoint.recalibrateIMU();


        hardware.pivot.setPosition(0.5);

        final double CLAW_CLOSE = 0.2;
        final double CLAW_OPEN = 0.5;

        scheduler.add(new OneShot(() -> {
                    hardware.backLeft.setPower(0.25);
                    hardware.backRight.setPower(0.25);
                    hardware.frontRight.setPower(0.25);
                    hardware.frontLeft.setPower(0.25);
                }))
//                .then(new WaitUntil(() -> {
////                    Pose2D pose2D = hardware.PinPoint.getPosition();
////                    hardware.PinPoint.update();
////                    return pose2D.getX(DistanceUnit.INCH) < -36;
//
//                }))
                .then(new OneShot(() -> {
                    hardware.backLeft.setPower(-0.25);
                    hardware.backRight.setPower(0.25);
                    hardware.frontRight.setPower(-0.25);
                    hardware.frontLeft.setPower(0.25);
                }))
//                .then(new WaitUntil(() -> {
////                    Pose2D pose2D = hardware.PinPoint.getPosition();
////                    hardware.PinPoint.update();
////                    return pose2D.getY(DistanceUnit.INCH) > 36;
//                }))
                .then(new OneShot(() -> {
                    hardware.backLeft.setPower(0);
                    hardware.backRight.setPower(0);
                    hardware.frontRight.setPower(0);
                    hardware.frontLeft.setPower(0);
                }));
        scheduler.add(new OneShot(() -> {
                    hardware.claw.setPosition(CLAW_CLOSE);
                }))
                .then(Wait.s(0.5))
                .then(new OneShot(() -> {
                    hardware.claw.setPosition(CLAW_OPEN);
                }))
                .then(Wait.s(0.5))
                .then(new OneShot(() -> {
                    hardware.claw.setPosition(CLAW_CLOSE);
                }))
                .then(Wait.s(0.5))
                .then(new OneShot(() -> {
                    hardware.claw.setPosition(CLAW_OPEN);
                }));
        scheduler.add(new WaitUntil(() -> gamepad1.a))
                .then(new OneShot(() -> {
                    hardware.pivot.setPosition(0.1);
                }));


        waitForStart();

        while (opModeIsActive()) scheduler.tick();
    }
}
