package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.BOTTOM_STOP_STOWED;
import static org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_UP;
import static org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.OUTTAKE_POWER;
import static org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_DOWN;
import static org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_UP;
import static org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SLIDER_OUT;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;
import org.firstinspires.ftc.teamcode.systems.Combo;
import org.firstinspires.ftc.teamcode.tasks.WaitUntilContinuous;
import org.firstinspires.ftc.teamcode.utilities.StaticStore;

import java.util.List;

import io.github.gearup12499.taskshark.FastScheduler;
import io.github.gearup12499.taskshark.prefabs.Group;
import io.github.gearup12499.taskshark.prefabs.OneShot;
import io.github.gearup12499.taskshark.prefabs.Wait;

@TeleOp

public class ShooterTest3 extends LinearOpMode {

    CompBot2Hardware hardware;
    FastScheduler sch;

    @Override

    public void runOpMode() throws InterruptedException {
        hardware = new CompBot2Hardware(hardwareMap);
        sch = new FastScheduler();

        hardware.initMotion();

        hardware.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        hardware.limelight.setPollRateHz(100);
        hardware.limelight.pipelineSwitch(2);
        hardware.limelight.start();

        hardware.flipper.setPosition(FLIPPER_DOWN);
        hardware.slider.setPosition(SLIDER_OUT);
        hardware.bottomBallStop.setPosition(0.42);

        boolean wasb = false;
        boolean wasx = false;
        boolean wasdpad = false;

        //  double ticks_per_degree = TURRET_CW_90 / 90.0;
        double targetvel = 1200;
        double targetpos = 0.182;

        waitForStart();

        while (opModeIsActive()) {

            double x = hardware.pinpoint.getPosX(DistanceUnit.INCH);
            double y = hardware.pinpoint.getPosY(DistanceUnit.INCH);
            double thetaD = hardware.pinpoint.getHeading(AngleUnit.DEGREES);
            double thetaR = hardware.pinpoint.getHeading(AngleUnit.RADIANS);

            LLResult result = hardware.limelight.getLatestResult();

            hardware.copyShooterPower();

            if (gamepad1.dpad_up) {
                hardware.flipper.setPosition(0.68);
                sleep(1000);
                hardware.setIntakePower(-0.8);

                hardware.flipper.setPosition(0.25);
                sleep(500);
                hardware.setIntakePower(1);
            }
            if (gamepad1.dpad_right && !wasdpad) {
                targetpos += 0.094;

                if (targetpos > 0.56) {
                    targetpos = 0.182;
                }
                hardware.hood.setPosition(targetpos);
            }

            if (gamepad1.dpad_down) {
                hardware.bottomBallStop.setPosition(0.58);
            }

            if (gamepad1.dpad_left) {
                hardware.bottomBallStop.setPosition(0.15);
                sleep(500);
                hardware.flipper.setPosition(0.68);
                sleep(800);
                hardware.setIntakePower(-0.8);

                hardware.flipper.setPosition(0.25);
                sleep(500);
                hardware.setIntakePower(1);
            }
            if (gamepad1.a) {
                hardware.setShoot1Vel(targetvel);
                hardware.setIntakePower(1);
            }

            if (gamepad1.b && !wasb) {
                targetvel += 20;
                hardware.setShoot1Vel(targetvel);
            }

            if (gamepad1.x && !wasx) {
                targetvel -= 20;
                hardware.setShoot1Vel(targetvel);
            }

            if (gamepad1.y) {
                hardware.setShoot1Vel(0);
                hardware.setIntakePower(0);
            }

            // hold LB to intake w/ shooter stop
            if (gamepad1.leftBumperWasPressed()) {
                hardware.flipper.setPosition(FLIPPER_DOWN);
                hardware.shooterBallStop.setPosition(SHOOTER_STOP_DOWN);
                hardware.setIntakePower(0.8);
            }
            if (gamepad1.leftBumperWasReleased()) {
                hardware.setIntakePower(0.0);
                hardware.shooterBallStop.setPosition(SHOOTER_STOP_UP);
            }

            if (gamepad1.rightBumperWasPressed()) {
                sch.add(new Group(it -> {
                }) {{
                    // reimplementation of Combo.shoot + Combo.shootAfter
                    // combine with pressing X / B to change speed
                    getScheduler()
                            .add(new OneShot(() -> {
                                hardware.setIntakePower(1.0);
                                hardware.bottomBallStop.setPosition(BOTTOM_STOP_STOWED);
                                hardware.shooterBallStop.setPosition(SHOOTER_STOP_UP);
                                hardware.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
                            }))
                            .then(new WaitUntilContinuous(0.3, () -> !hardware.frontRamp.getState() && (hardware.colorBottomLeft.getDistance(DistanceUnit.MM) < 110.0
                                    || hardware.colorBottomRight.getDistance(DistanceUnit.MM) < 110.0)))
                            .then(new OneShot(() -> hardware.flipper.setPosition(FLIPPER_UP)))
                            .then(Wait.ms(700))
                            .then(new OneShot(() -> {
                                hardware.setIntakePower(OUTTAKE_POWER);
                                hardware.flipper.setPosition(FLIPPER_DOWN);
                            }))
                            .then(Wait.ms(500))
                            .then(new OneShot(() -> {
                                hardware.setIntakePower(0.0);
                                hardware.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard);
                            }));
                    this.require(CompBot2Hardware.Locks.INTAKE_STORAGE);
                }});
            }

            wasb = gamepad1.b;
            wasx = gamepad1.x;
            wasdpad = gamepad1.dpad_right;

            double currentVel = hardware.getShoot1Vel();
            double hoodpos = hardware.gethoodpos();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("target velocity", targetvel);
            telemetry.addData("Current Vel: ", currentVel);
            telemetry.addData("at target", Math.abs(targetvel - currentVel) < 20);
            telemetry.addData("hood position", hoodpos);
            telemetry.addData("PosX", x);
            telemetry.addData("PosY", y);
            telemetry.addData("Degrees", thetaD);
            telemetry.addData("Radians", thetaR);
            telemetry.update();

            hardware.pinpoint.update();
            sch.tick();
        }
    }
}