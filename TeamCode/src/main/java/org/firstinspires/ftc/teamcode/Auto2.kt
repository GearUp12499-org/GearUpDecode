package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.Combo
import org.firstinspires.ftc.teamcode.systems.Prismatic
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.tasks.Deferred
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.utilities.StaticStore

abstract class Auto2(private val red: Boolean) : LinearOpMode() {
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE

    private lateinit var hw: CompBot2Hardware
    private lateinit var shooter: ShooterImpl

    private var skipExtra = false

    fun reconfigure(skip: Boolean) {
        skipExtra = skip
        Prismatic.configurationLights(
            hw.prism,
            red,
            if (skip) Prismatic.Mode.ALTERNATE else Prismatic.Mode.MAIN
        )

        telemetry.addLine("AUTO SETUP --------")
        telemetry.addLine("Skip 2nd Spike Line: ${if (skip) "YES (ALTERNATE)" else "NO (MAIN)"}")
        telemetry.addLine("Press 1/RB to change")
        telemetry.update()
    }

    override fun runOpMode() {
        TaskSharkAndroid.setup()

        hw = CompBot2Hardware(hardwareMap)
        hw.dropDown.position = CompBot2Hardware.DROP_DOWN_BOTTOM
        hw.slider.position = CompBot2Hardware.SLIDER_IN
        hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_BALL_STOP
        hw.hood.position = CompBot2Hardware.HOOD_UP
        hw.pinpoint.setPosition(poseSet.farStart.asPose2D)

        StaticStore.fallbackArtboard = if (red) Artboard.ARTBOARD_0 else Artboard.ARTBOARD_1
        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)

        hw.turret.targetPosition = 0
        hw.turret.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.turret.power = 1.0

        val sch = FastScheduler()
        shooter = sch.add(ShooterImpl(hw))
        sch.add(Configurator())

        reconfigure(false)

        val startFlag = sch.add(SentinelTask())

        sch.add(object : Task.Anonymous() {
            override fun onTick(): Boolean {
                hw.pinpoint.update()
                return false
            }
        })

        startFlag.then(VirtualGroup {
            add(REmover.drive2Pose2(hw, poseSet.farShoot))
            add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.3))
        })
            .then(Combo.shoot(hw, shooter, 0.5))
            .then(shooter.setTargetAsync(0.0))
            .then(VirtualGroup {
                val intake = add(Combo.intake(hw))
                add(REmover.drive2Pose2(hw, poseSet.set3pos))
                    .then(REmover.drive2Pose2(hw, poseSet.set3out))
                    .then(VirtualGroup {
                        add(REmover.drive2Pose2(hw, poseSet.farShoot))
                        add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.2))
                    })
                    .then(OneShot {
                        intake.finish()
                    })
            })
            .then(Combo.shoot(hw, shooter, 0.5))
            .then(Deferred {
                if (skipExtra) null
                else VirtualGroup {
                    val intake = add(Combo.intake(hw))
                    add(REmover.drive2Pose2(hw, poseSet.set4pos))
                        .then(REmover.drive2Pose2(hw, poseSet.set4out, 0.35))
                        .then(VirtualGroup {
                            add(REmover.drive2Pose2(hw, poseSet.farShoot))
                            add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.2))
                        })
                        .then(OneShot {
                            intake.finish()
                        })
                }
            }).then(Deferred {
                if (skipExtra) null
                else Combo.shoot(hw, shooter, 0.5)
            }).then(REmover.drive2Pose2(hw, poseSet.auto2park))

        while (opModeInInit()) sch.tick()

        startFlag.finish()

        while (opModeIsActive()) sch.tick()

        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
        StaticStore.mark() // indicate to carry pinpoint into teleop in the next 30 seconds
    }

    private inner class Configurator : Task<Configurator>() {
        private var rbt = false

        override fun onTick(): Boolean {
            if (opModeIsActive()) finish()

            val rb = gamepad1.right_bumper
            if (rb && !rbt) {
                reconfigure(!skipExtra)
            }

            rbt = rb

            return false // use finish() to kill this
        }
    }
}