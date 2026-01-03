package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMapper;

@TeleOp
public class ServoSteper extends LinearOpMode {

    CompBot2Hardware hardware;
    double position = 0.5;

    double maxServo = 0.9;
    double minServo = 0.1;

    int MotorPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new CompBot2Hardware(hardwareMap);

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
            hardware.bottomBallStop.setPosition(position);

            if (gamepad1.a){
                hardware.intake.setPower(1);
            }
            else {
                hardware.intake.setPower(0);
            }

            if (gamepad1.right_trigger > 0.5){
                MotorPosition += 10;
            }

            if (gamepad1.left_trigger > 0.5){
                MotorPosition -= 10;
            }




            telemetry.addData("motorPosition",hardware.turret.getCurrentPosition());
            telemetry.addData("position",position);
            telemetry.update();

            sleep(50);
        }

    }
}
