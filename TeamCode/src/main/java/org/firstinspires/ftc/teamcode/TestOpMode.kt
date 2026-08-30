package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.BOTTOM_STOP_STOWED
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SLIDER_IN
@TeleOp
class TestOpMode: LinearOpMode(){
    private lateinit var hw: CompBot2Hardware
    override fun runOpMode() {
        hw = CompBot2Hardware(hardwareMap)
        val intakeMachine = IntakeMachine(hw)
        hw.bottomBallStop.position = BOTTOM_STOP_STOWED
        hw.slider.position = SLIDER_IN
        waitForStart()
        while(opModeIsActive()){
            intakeMachine.update(gamepad1.a, gamepad1.b)
        }
    }
}
