package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware

@TeleOp(name = "go forward")
@Disabled
class GoForward : LinearOpMode() {
    override fun runOpMode() {
        val hw = CompBotHardware(hardwareMap)
        waitForStart()
        hw.frontLeft.power = 0.3
        hw.frontRight.power = 0.3
        hw.backLeft.power = 0.3
        hw.backRight.power = 0.3
        while (opModeIsActive()) sleep(0)
    }
}