package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.ShooterImpl

@TeleOp
class IntakeTest : LinearOpMode() {
    lateinit var hw: CompBot2Hardware
    lateinit var shooter: ShooterImpl
    override fun runOpMode() {
        val s = FastScheduler()
        hw = CompBot2Hardware(hardwareMap)
        hw.initMotion()
        shooter = s.add(ShooterImpl(hw))
        s.add(OneShot {
            shooter.setTarget(1600.0)
        })
        waitForStart()

        var da = false
        while (opModeIsActive()) {
            s.tick()
            val a = gamepad1.a
            if (a && !da) s.add(doTheIntakeThing())
            da = a
        }
    }

    fun doTheIntakeThing() = Group {
        // prime the system
        it.add(OneShot {
            hw.intake.power = 0.0
            hw.slider.position = CompBot2Hardware.SLIDER_IN
            hw.dropDown.position = CompBot2Hardware.DROP_DOWN_SWEET_SPOT
            hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
        })
            .then(Wait.ms(250))
            .then(OneShot {
                hw.intake.power = CompBot2Hardware.INTAKE_POWER
            })
            .then(WaitUntil {
                hw.colorTopLeft.getDistance(DistanceUnit.MM) < 110.0
            })
            .then(OneShot {
                hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_BALL_STOP
            })
            .then(WaitUntil {
                hw.frontRamp.state && hw.middleRamp.state
            })
            .then(OneShot {
                hw.intake.power = 0.0
                hw.dropDown.position = CompBot2Hardware.DROP_DOWN_BOTTOM
            })
    }
}