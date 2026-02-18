package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

@TeleOp
public class ServoSteper extends LinearOpMode {

    CompBot2Hardware hardware;
    double position = CompBot2Hardware.SHOOTER_STOP_DOWN;

    double maxServo = 1.0;
    double minServo = 0.0;

    int MotorPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new CompBot2Hardware(hardwareMap);

        Servo theServo = hardware.shooterBallStop;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper){
                position += 0.01;
                if(position > maxServo){
                    position = maxServo;
                }
            }

            if (gamepad1.left_bumper){
                position -= 0.01;
                if(position < minServo){
                    position = minServo;
                }
            }
            theServo.setPosition(position);

            if (gamepad1.a){
                hardware.setIntakePower(1);
            }
            else {
                hardware.setIntakePower(0);
            }

            if (gamepad1.right_trigger > 0.5){
                MotorPosition += 10;
            }

            if (gamepad1.left_trigger > 0.5){
                MotorPosition -= 10;
            }




//            telemetry.addData("motorPosition", hardware.turret.getCurrentPosition());
            telemetry.addData("position",position);
            telemetry.update();

            sleep(50);
        }

    }
}
