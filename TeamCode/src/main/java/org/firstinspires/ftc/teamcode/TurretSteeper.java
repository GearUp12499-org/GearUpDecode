package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.systems.TurretImpl;

@TeleOp
public class TurretSteeper extends LinearOpMode {
    private double negLimit;
    private double posLimit;
    CRServo servoTurret1;
    CRServo servoTurret2;
    DcMotorEx intake2;
    double basePower = 0.1;
    double P = TurretImpl.P;
    double I = TurretImpl.I;
    double D = TurretImpl.D;
    double maxI = 20000.0;
    double deadbandTicks = 136.0;
    double minPowerErrorTicks = 320.0;
    double iZoneTicks = 650.0;
    double nearTargetIClamp = 2500.0;
    double integralError = 0.0;
    double prevError = 0.0;
    long lastPidTime = 0L;
    boolean resetPid = true;

    private static final double TICKS_PER_DEGREE = 67.9;

    private void turnTurret(double power, CRServo t1, CRServo t2){
        double appliedpower = -power;
        t1.setPower(appliedpower);
        t2.setPower(appliedpower);
    }

    private double limit(int currentPos, double power){
        if(currentPos >= posLimit && power > 0){
            return 0.0;
        }
        if(currentPos <= negLimit && power < 0) {
            return 0.0;
        }
        return power;
    }

    private void setTurnPosition(double deg){
        double targetTicks = -deg * TICKS_PER_DEGREE;
        if (targetTicks > posLimit) {
            targetTicks = posLimit;
        } else if (targetTicks < negLimit) {
            targetTicks = negLimit;
        }
        double error = targetTicks - intake2.getCurrentPosition();
        long now = System.nanoTime();
        double dt = 0.0;
        if (lastPidTime != 0L) {
            dt = (now - lastPidTime) / 1e9;
        }
        lastPidTime = now;

        if (Math.abs(error) <= deadbandTicks) {
            prevError = error;
            integralError = 0.0;
            this.turnTurret(0.0, servoTurret1, servoTurret2);
            return;
        }

        if (resetPid) {
            prevError = error;
            integralError = 0.0;
            resetPid = false;
        }

        if (error * prevError < 0.0) {
            integralError = 0.0;
        }

        if (Math.abs(error) <= iZoneTicks) {
            integralError += error * dt;
            double activeIClamp = Math.abs(error) <= minPowerErrorTicks ? nearTargetIClamp : maxI;
            if (integralError > activeIClamp) {
                integralError = activeIClamp;
            } else if (integralError < -activeIClamp) {
                integralError = -activeIClamp;
            }
        } else {
            integralError *= 0.9;
        }

        double derivative = 0.0;
        if (dt > 0.0) {
            derivative = (error - prevError) / dt;
        }
        prevError = error;


        telemetry.addData("P * error", (P * error));
        telemetry.addData("I * error", (I * integralError));
        telemetry.addData("D * error", (D * derivative));
        Log.i("!", String.format("%.2f %.2f %.2f PID", P*error, I*integralError, D*derivative));
        telemetry.update();
        double output = (P * error) + (I * integralError) + (D * derivative);
        if(output > 1.0){
            output = 1.0;
        } else if (output < -1.0) {
            output = -1.0;
        }
        if (Math.abs(error) > minPowerErrorTicks && Math.abs(output) < basePower) {
            output = Math.copySign(basePower, output);
        }
        this.turnTurret(limit(intake2.getCurrentPosition(), output), servoTurret1, servoTurret2);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        servoTurret1 = hardwareMap.get(CRServo.class, "turret1");
        servoTurret2 = hardwareMap.get(CRServo.class, "turret2");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        negLimit = -9400.0;
        posLimit = 9400.0;
        double servoPower = 1;

        waitForStart();

        while (opModeIsActive()) {
                this.setTurnPosition(90.0);
            }
        }

    }
