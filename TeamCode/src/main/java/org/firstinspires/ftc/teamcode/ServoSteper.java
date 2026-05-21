package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;

@TeleOp
public class ServoSteper extends LinearOpMode {

    CompBot2Hardware hardware;
    double position = 0.57;

    double maxServo = 1.0;
    double minServo = 0.0;

    double deltaPosition = 0;

    double deltaPosition2 = 0;

    int MotorPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new CompBot2Hardware(hardwareMap);

        Servo theServo = hardware.shooterBallStop;

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.right_bumper){
                deltaPosition += 0.01;
                if(position > maxServo){
                    position = maxServo;
                }
            }

            if (gamepad1.left_bumper){
                deltaPosition -= 0.01;
                if(position < minServo){
                    position = minServo;
                }
            }

            if (gamepad1.a){
                deltaPosition2 -= 0.01;
                if(position < minServo){
                    position = minServo;
                }
            }

            if (gamepad1.b){
                deltaPosition2 += 0.01;
                if(position < minServo){
                    position = minServo;
                }
            }
            hardware.leftKickstand.setPosition(position + deltaPosition);
            hardware.rightKickstand.setPosition(position + deltaPosition2);

//            if (gamepad1.a){
//                hardware.setIntakePower(1);
//            }
//            else {
//                hardware.setIntakePower(0);
//            }

            if (gamepad1.right_trigger > 0.5){
                MotorPosition += 10;
            }

            if (gamepad1.left_trigger > 0.5){
                MotorPosition -= 10;
            }




//            telemetry.addData("motorPosition", hardware.turret.getCurrentPosition());
            telemetry.addData("positionleft",position+deltaPosition);
            telemetry.addData("positionright",position+deltaPosition2);
            telemetry.update();

            sleep(50);
        }

    }
}
