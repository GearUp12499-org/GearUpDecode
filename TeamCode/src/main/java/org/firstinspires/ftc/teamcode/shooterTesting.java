package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;

@TeleOp
public class shooterTesting extends LinearOpMode {

    CompBotHardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new CompBotHardware(hardwareMap);

        waitForStart();

        double velocity = 0;

        while(opModeIsActive()){

            if (gamepad2.right_trigger > 0.5){
                velocity += 1;
            }

            if (gamepad2.left_trigger > 0.5){
                velocity -= 1;
            }

            if (gamepad2.b) {
                hardware.flipper.setPosition(CompBotHardware.FLIPPER_UP);
            } else {
                hardware.flipper.setPosition(CompBotHardware.FLIPPER_DOWN);
            }

            if (gamepad2.right_bumper) {
                // 2000, 1500: too fast for mid range
                // 1000: too slow for mid range
                // 1500 for far zone
                hardware.shooter1.setVelocity(velocity);
            } else if (gamepad2.left_bumper) {
                hardware.shooter1.setVelocity(-500);
            } else {
                hardware.shooter1.setVelocity(0);
            }

            telemetry.addData("velocity", velocity);
            telemetry.addData("actualVelocity", hardware.shooter1.getVelocity());
            telemetry.update();

        }

    }
}
