package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.Disabled
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
import org.firstinspires.ftc.teamcode.tasks.WaitUntilContinuous

@Disabled
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
            shooter.setTarget(1500.0)
        })
        waitForStart()

        var da = false
        var db = false
        while (opModeIsActive()) {
            s.tick()
            val a = gamepad1.a
            val b = gamepad1.b
            if (a && !da) s.add(doTheIntakeThing())
            if (b && !db) s.add(doTheOuttakeThing())
            da = a
            db = b
        }
    }

    fun doTheIntakeThing() = Group {
        // prime the system
        it.add(OneShot {
            hw.intake.power = 0.0
            hw.slider.position = CompBot2Hardware.SLIDER_IN
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
            .then(WaitUntilContinuous(.3) {
                hw.frontRamp.state && hw.middleRamp.state
            })
            .then(OneShot {
                hw.intake.power = 0.0
            })
    }

    fun doTheOuttakeThing() = Group {
        it.add(OneShot {
            hw.intake.power = 1.0
            hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
        })
            .then(Wait.ms(750))
            .then(OneShot {
                hw.flipper.position = CompBot2Hardware.FLIPPER_UP
            })
            .then(Wait.ms(800))
            .then(OneShot {
                hw.flipper.position = CompBot2Hardware.FLIPPER_DOWN
                hw.intake.power = CompBot2Hardware.OUTTAKE_POWER
            })
            .then(Wait.ms(500))
            .then(OneShot {
                hw.intake.power = 0.0
            })
    }
}