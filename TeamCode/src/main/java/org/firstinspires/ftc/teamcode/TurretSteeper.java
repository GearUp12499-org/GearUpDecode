package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TurretSteeper extends LinearOpMode {
    private double negLimit;
    private double posLimit;
    CRServo servoTurret1;
    CRServo servoTurret2;
    DcMotorEx intake2;
    double basePower = 0.3; // needs to be tested - this is the power to barely move the turret
    double P = 0.0025;

    private static final double TICKS_PER_DEGREE = (double) 67.9;

    private void turnTurret(double power, CRServo t1, CRServo t2){
        t1.setPower(power);
        t2.setPower(power);
    }

    private double limit(int currentPos, double power){
        // IMPORTANT: positive position is in the direction of negative power (negative power makes the turret turn in a pos direction)
        telemetry.addData("Current Pos: ", currentPos);
        telemetry.addData("Current Power: ", power);
        telemetry.update();
        if(currentPos > posLimit && power < 0){
            return 0.0;
        }
        if(currentPos < negLimit && power > 0) {
            return 0.0;
        }
        return power;
    }

    private void setTurnPosition(double deg){ // NOT WORKING YET - NEED TO FIX SIGNS
        double targetTicks = deg * TICKS_PER_DEGREE;
        double error = targetTicks - intake2.getCurrentPosition();
        double output = P * error;
        if(output > 1){
            output = 1;
        }
        if(Math.abs(error) <= 136.0 && Math.abs(output) < basePower) {
            output = 0;
        }
        this.turnTurret(limit(intake2.getCurrentPosition(), output), servoTurret1, servoTurret2);

    }

    // ticks = -6155 --> 90 degrees
    @Override
    public void runOpMode() throws InterruptedException {
        servoTurret1 = hardwareMap.get(CRServo.class, "turret1");
        servoTurret2 = hardwareMap.get(CRServo.class, "turret2");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        negLimit = -9400.0;
        posLimit = 9400.0;
        double servoPower = 1;

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a){
                this.turnTurret(limit(intake2.getCurrentPosition(), servoPower), servoTurret1, servoTurret2);
            } else if(gamepad1.b){
                this.turnTurret(limit(intake2.getCurrentPosition(), -servoPower), servoTurret1, servoTurret2);
            } else if(gamepad1.x){
                this.setTurnPosition(90.0);
            }
            else {
                this.turnTurret(0.0, servoTurret1, servoTurret2);
            }
//            telemetry.addData("Current Position (Ticks): ", intake2.getCurrentPosition());
//            telemetry.update();
        }

    }
}
