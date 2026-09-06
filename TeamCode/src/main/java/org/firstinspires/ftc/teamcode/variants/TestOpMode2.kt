package org.firstinspires.ftc.teamcode.variants

import androidx.annotation.RestrictTo
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.prefabs.OneShot
import org.firstinspires.ftc.teamcode.IntakeTaskShark
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
@TeleOp
class TestOpMode2: LinearOpMode() {
    override fun runOpMode() {
        val sch: FastScheduler = FastScheduler()
        val hw: CompBot2Hardware = CompBot2Hardware(hardwareMap)
        var was_A = false
        waitForStart()
        while (opModeIsActive()) {
            sch.tick()
            if(gamepad1.a && was_A == false){
                was_A = true
                sch.add(IntakeTaskShark.intake(hw))
                    .then(IntakeTaskShark.intakeAfter(hw))
                    .then(OneShot{
                        was_A = false
                    })
            }
        }
    }
    private inner class InOut
}