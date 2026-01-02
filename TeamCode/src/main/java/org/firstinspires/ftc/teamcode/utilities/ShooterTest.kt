package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.ShooterImpl

@TeleOp
class ShooterTest : LinearOpMode() {
    override fun runOpMode() {
        TaskSharkAndroid.setup()
        val hw = CompBot2Hardware(hardwareMap)
        val sch = FastScheduler()

        hw.initMotion()

        hw.pinpoint.resetPosAndIMU()

        sch.add(object : Task.Anonymous() {
            override fun onTick(): Boolean {
                hw.pinpoint.update()
                return false
            }
        })

        val shooter = sch.add(ShooterImpl(hw))
        while (opModeInInit()) {
            hw.pinpoint.update()
        }

        var wasA = false
        var wasB = false
        var wasY = false
        var wasRB = false
        var wasLB = false

        hw.intake.power = 1.0

        while (opModeIsActive()) {
            sch.tick()

            val isA = gamepad1.a
            val isB = gamepad1.b
            val isY = gamepad1.y
            if (isA && !wasA) {
                shooter.setTarget(1600.0)
            }
            if (isB && !wasB) {
                shooter.setTarget(1200.0)
            }
            if (isY && !wasY) {
                shooter.setTarget(0.0)
            }
            wasA = isA
            wasB = isB
            wasY = isY

            val isRB = gamepad1.right_bumper
            val isLB = gamepad1.left_bumper
            if (isRB && !wasRB) {
                hw.flipper.position = CompBot2Hardware.FLIPPER_UP
            }
            if (isLB && !wasLB) {
                sch
                    .add(OneShot {
                        hw.intake.power = -0.8
                        hw.flipper.position = CompBot2Hardware.FLIPPER_DOWN
                    })
                    .then(Wait.s(0.5))
                    .then(OneShot {
                        hw.intake.power = 1.0
                    })
            }

            wasRB = isRB
            wasLB = isLB
        }
    }
}