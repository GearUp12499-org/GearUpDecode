package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import java.lang.Math.clamp

/**
 * steep the servo
 */
@TeleOp
class ServoSteper : LinearOpMode() {
    companion object {
        const val SPEED = 0.25 // per second
    }

    override fun runOpMode() {
        val theServo: Servo = fromHardwareCls { flipper }
//        val theServo: Servo = fromName("flipper")

        waitForStart()
        val runtime = ElapsedTime(ElapsedTime.Resolution.SECONDS)
        var lastTime = 0.0
        var pos = 0.5

        while (opModeIsActive()) {
            val now = runtime.time()
            val delta = now - lastTime
            lastTime = now

            val bumper1 = gamepad1.left_bumper
            val bumper2 = gamepad1.right_bumper
            val mod = when {
                (bumper1 && bumper2) -> 0.25
                (bumper1 || bumper2) -> 0.5
                else -> 1.0
            }
            val speed = SPEED * mod

            if (gamepad1.dpad_up) {
                pos += delta * speed
            } else if (gamepad1.dpad_down) {
                pos -= delta * speed
            }
            pos = clamp(pos, 0.0, 1.0)

            theServo.position = pos
            telemetry.addData("Position", pos)
            telemetry.addData("Speed (per s)", speed)
            telemetry.addData("Speed Mod", mod)
            telemetry.addLine("Press dpad up/down to change")
            telemetry.addLine("Press left/right bumper to slow")
            telemetry.update()
        }
    }

    private inline fun <reified T> fromName(name: String): T {
        return hardwareMap.get(T::class.java, name)
    }

    private inline fun <T> fromHardwareCls(getIt: CompBotHardware.() -> T): T {
        val instance = CompBotHardware(hardwareMap)
        return instance.getIt()
    }
}