package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.REmover.RobotPose
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.tasks.WaitUntilContinuous

@Autonomous
class Auto1 : LinearOpMode() {
    private lateinit var hw: CompBot2Hardware
    private lateinit var shooter: ShooterImpl

    override fun runOpMode() {
        TaskSharkAndroid.setup()

        hw = CompBot2Hardware(hardwareMap)
        hw.dropDown.position = CompBot2Hardware.DROP_DOWN_BOTTOM
        hw.slider.position = CompBot2Hardware.SLIDER_IN
        hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_BALL_STOP
        hw.hood.position = CompBot2Hardware.HOOD_50
        hw.pinpoint.setPosition(
            Pose2D(
                DistanceUnit.INCH,
                -63.375,
                -17.25,
                AngleUnit.RADIANS,
                -Math.PI
            )
        )

        val sch = FastScheduler()
        shooter = sch.add(ShooterImpl(hw))

        sch.add(object : Task.Anonymous() {
            override fun onTick(): Boolean {
                hw.pinpoint.update()
                return false
            }
        })

        sch.add(OneShot {
            shooter.setTarget(1290.0)
        })

        sch.add(REmover.drive2Pose2(hw, RobotPose(24.0, -24.0, 3 * Math.PI / 4)))
            .then(doTheOuttakeThing())
            .then(VirtualGroup {
                add(doTheIntakeThing())
                add(REmover.drive2Pose2(hw, RobotPose(12.0, -30.375, -Math.PI / 2)))
                    .then(REmover.drive2Pose2(hw, RobotPose(12.0, -54.125, -Math.PI / 2)))
            })
            .then(REmover.drive2Pose2(hw, RobotPose(24.0, -24.0, 3 * Math.PI / 4)))
            .then(doTheOuttakeThing())
            .then(VirtualGroup {
                add(doTheIntakeThing())
                add(REmover.drive2Pose2(hw, RobotPose(-12.0, -30.375, -Math.PI / 2)))
                    .then(REmover.drive2Pose2(hw, RobotPose(-12.0, -61.625, -Math.PI / 2)))
            })
            .then(REmover.drive2Pose2(hw, RobotPose(24.0, -24.0, 3 * Math.PI / 4)))
            .then(doTheOuttakeThing())
            .then(VirtualGroup {
                add(doTheIntakeThing())
                add(REmover.drive2Pose2(hw, RobotPose(-36.0, -30.375, -Math.PI / 2)))
                    .then(REmover.drive2Pose2(hw, RobotPose(-36.0,  -61.625, -Math.PI / 2)))
            })
            .then(REmover.drive2Pose2(hw, RobotPose(24.0, -24.0, 3 * Math.PI / 4)))
            .then(doTheOuttakeThing())

        waitForStart()
        while (opModeIsActive()) sch.tick()
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
            .then(WaitUntilContinuous(.3) {
                hw.frontRamp.state && hw.middleRamp.state
            })
            .then(OneShot {
                hw.intake.power = 0.0
                hw.dropDown.position = CompBot2Hardware.DROP_DOWN_BOTTOM
            })
    }

    fun doTheOuttakeThing() = Group {
        it.add(OneShot {
            hw.intake.power = 1.0
            hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
        })
            .then(Wait.ms(1250))
            .then(OneShot {
                hw.flipper.position = CompBot2Hardware.FLIPPER_UP
            })
            .then(Wait.ms(1000))
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