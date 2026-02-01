package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.Combo
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.tasks.Deferred
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.tasks.compose
import org.firstinspires.ftc.teamcode.utilities.StaticStore

abstract class Auto2(private val red: Boolean) : LinearOpMode() {
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE

    private lateinit var hw: CompBot2Hardware
    private lateinit var shooter: ShooterImpl

    private var skipExtra = false

    fun reconfigure(skip: Boolean, prismBroken: Boolean) {
        skipExtra = skip
        hw.refreshPrismState()

        if (prismBroken) {
            telemetry.addLine("PRISM IS DISABLED")
            telemetry.addLine()
        }

        telemetry.addLine(
            "<big><big>This is a " +
                    "<font color=\"${if (red) "#ff4040" else "#00ffff"}\"><strong>${if (red) "RED" else "BLUE"}</strong></font>" +
                    " auto</big></big>"
        )
//        telemetry.addLine("Collect Corner Artifacts: ${if (skip) "<strong>NO (ALTERNATE)</strong>" else "YES (MAIN)"}")
//        telemetry.addLine("Press 1/RB to change")
        telemetry.addLine("Press 1/X to toggle Prism")
        telemetry.update()
    }

    override fun runOpMode() {
        TaskSharkAndroid.setup()

        hw = CompBot2Hardware(hardwareMap)
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

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        telemetry.update()
        reconfigure(false, StaticStore.prismBroken)

        val startFlag = sch.add(SentinelTask())

        sch.add(compose {
            var state: GoBildaPinpoint2Driver.DeviceStatus? = null
            onTick {
                val stateNew = hw.pinpoint.deviceStatus
                if (stateNew != state) {
                    Log.w("Pinpoint", "state changed $state -> $stateNew")
                    state = stateNew
                }
                hw.pinpoint.update()
                false
            }
        })

        startFlag.then(VirtualGroup {
            add(REmover.drive2Pose2(hw, poseSet.farShoot))
            add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.3))
        })
            .then(Combo.shoot(hw, shooter, 0.5))
            .then(VirtualGroup {
                val intake = add(Combo.intake(hw, 1.0))
                add(REmover.drive2Pose2(hw, poseSet.set4pos))
                    .then(REmover.drive2Pose2(hw, poseSet.set4out, 0.35))
                    .then(Wait.s(2))
                    .then(REmover.drive2Pose2(hw, poseSet.set4out2, 0.35))
                    .then(Wait.s(2))
                    .then(VirtualGroup {
                        add(REmover.drive2Pose2(hw, poseSet.farShoot))
                        add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.2))
                    })
                    .then(OneShot {
                        intake.finish()
                    })
            })
            .then(Combo.shoot(hw, shooter, 0.5))
            .then(REmover.drive2Pose2(hw, poseSet.auto2park))

        while (opModeInInit()) sch.tick()

        startFlag.finish()

        while (opModeIsActive()) sch.tick()

        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
        StaticStore.mark() // indicate to carry pinpoint into teleop in the next 30 seconds
    }

    private inner class Configurator : Task<Configurator>() {
        private var rbt = false
        private var xt = false

        override fun onTick(): Boolean {
            if (opModeIsActive()) finish()

            val rb = gamepad1.right_bumper
            val x = gamepad1.x
            if (rb && !rbt) {
                reconfigure(!skipExtra, StaticStore.prismBroken)
            }
            if (x && !xt) {
                reconfigure(skipExtra, !StaticStore.prismBroken)
            }

            rbt = rb
            xt = x

            return false // use finish() to kill this
        }
    }
}