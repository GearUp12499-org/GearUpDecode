package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.CompBot2HardwareOld;

@Disabled
@TeleOp
// @Autonomous
public class ShooterPID extends LinearOpMode {
    CompBot2HardwareOld hardware;
    ElapsedTime runTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double kf = 0.0005;
    double currentTime = runTimer.time();
    double prevTime;
    double deltaTime;
    double sumError = 0;
    double prevError = 0;
    double pidf;

    double targetVel = 1100;
    double servoPos = 0.5; //hardware.axonEncoder.getVoltage();
    double hoodUp = 0.5589;
    double hoodDown = 0.1828;

    public void setBothPower(double power){
        hardware.shoot1.setPower(power);
        hardware.shoot2.setPower(power);
    }
    public void customSetVelocity(double targetVel){
        telemetry.addData("Target Vel: ", targetVel);
        currentTime = runTimer.time();
        deltaTime = Math.max(currentTime - prevTime, 0.001);

        double currentVel = hardware.shoot1.getVelocity();
        double currentError = targetVel - currentVel;
        telemetry.addData("Current Error: ", currentError);

        //Place holder for the i constant here
        pidf = kp*currentError + kd*(currentError - prevError)/deltaTime + ki*sumError + kf*targetVel;

        if(pidf > 1){
            pidf = 1;
        } else if(pidf < -1){
            pidf = -1;
        }

        telemetry.addData("Current Vel: ", currentVel);
        telemetry.addData("Output Power: ", pidf);
        this.setBothPower(pidf);

        prevTime = currentTime;
        prevError = currentError;
        telemetry.update();
    }

    public void shooterPIDController(double targetVel){
        double output = hardware.shoot1.getPower();
        hardware.shoot2.setPower(output);
        telemetry.addData("Output: ", output);
    }

    @Override
    public void runOpMode() {
        hardware = new CompBot2HardwareOld(hardwareMap);
        hardware.shoot1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(380, 40, 20, 0));


        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                targetVel++;
                hardware.shoot1.setVelocity(targetVel);
            } else if(gamepad1.left_bumper){
                targetVel--;
                hardware.shoot1.setVelocity(targetVel);
            }

            if(gamepad1.a){
                servoPos += 0.005;
                if(servoPos > hoodUp){
                    servoPos = hoodUp;
                }
                sleep(50);
//                hardware.axonServo.setPosition(servoPos);
            } else if(gamepad1.b){
                servoPos -= 0.005;
                if(servoPos < hoodDown){
                    servoPos = hoodDown;
                }
                sleep(50);
//                hardware.axonServo.setPosition(servoPos);
            }
//            telemetry.addData("Servo Voltage: ", hardware.axonEncoder.getVoltage());
//            telemetry.addData("Servo Position: ", hardware.axonServo.getPosition());
            telemetry.addData("Target Vel: ", targetVel);
            telemetry.addData("Vel: ", hardware.shoot1.getVelocity());
            telemetry.update();
//          customSetVelocity(1100);
            shooterPIDController(targetVel);

        }
    }
}
