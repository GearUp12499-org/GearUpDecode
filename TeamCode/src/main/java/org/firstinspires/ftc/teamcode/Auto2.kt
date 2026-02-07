package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.AprilTag
import org.firstinspires.ftc.teamcode.systems.Combo
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.tasks.PinpointSetupTask
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.tasks.compose
import org.firstinspires.ftc.teamcode.utilities.StaticStore
import org.firstinspires.ftc.vision.VisionPortal

abstract class Auto2(private val red: Boolean) : LinearOpMode() {
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE

    private lateinit var hw: CompBot2Hardware
    private lateinit var shooter: ShooterImpl
    private var pinpointSetupTask: PinpointSetupTask? = null
    private var aprilTag: AprilTag? = null
    private var confTask: ITask<*>? = null

    fun initLog() {
        if (StaticStore.prismBroken) {
            telemetry.addLine("PRISM IS DISABLED")
            telemetry.addLine()
        }

        telemetry.addLine(
            "<big><big>This is a " +
                    "<font color=\"${if (red) "#ff4040" else "#00ffff"}\"><strong>${if (red) "RED" else "BLUE"}</strong></font>" +
                    " auto</big></big>"
        )
        telemetry.addLine("Press 1/RB to recalibrate Pinpoint")
        telemetry.addLine("Press 1/X to toggle Prism")

        pinpointSetupTask?.let {
            telemetry.addLine()
            telemetry.addData(
                "Linear velo (in/s)",
                problem("%.6f".format(it.velocity), it.velocity < VEL_LIM)
            )
            telemetry.addData(
                "Angular velo (rad/s)",
                problem("%.6f".format(it.angularVelocity), it.angularVelocity < VEL_LIM)
            )
        }

        aprilTag?.let {
            telemetry.addLine()
            val state = it.visionPortal?.cameraState
            telemetry.addData(
                "Camera status",
                problem(state.toString(), state == VisionPortal.CameraState.STREAMING)
            )
        }

        telemetry.update()
    }

    fun reconfigure(prismBroken: Boolean, sch: Scheduler) {
        StaticStore.prismBroken = prismBroken
        hw.refreshPrismState()

        confTask?.stop()
        confTask = sch.add(Group {
            it.add(OneShot {
                hw.pinpoint.recalibrateIMU()
            }).then(WaitUntil {
                hw.pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY
            }).then(OneShot {
                hw.pinpoint.setPosition(poseSet.farStart.asPose2D)
            })
        })
    }

    override fun runOpMode() {
        TaskSharkAndroid.setup()

        hw = CompBot2Hardware(hardwareMap)
        hw.slider.position = CompBot2Hardware.SLIDER_IN
        hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
        hw.shooterBallStop.position = CompBot2Hardware.SHOOTER_STOP_UP
        hw.hood.position = CompBot2Hardware.HOOD_UP
        hw.pinpoint.setPosition(poseSet.farStart.asPose2D)

        StaticStore.fallbackArtboard = if (red) Artboard.ARTBOARD_0 else Artboard.ARTBOARD_1
        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)

        hw.turret.targetPosition = 0
        hw.turret.mode = DcMotor.RunMode.RUN_TO_POSITION
        hw.turret.power = 1.0

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        telemetry.update()

        val sch = FastScheduler()
        val startFlag = sch.add(SentinelTask())

        reconfigure(StaticStore.prismBroken, sch)
        sch.add(Configurator())
        pinpointSetupTask = sch.add(PinpointSetupTask(hw.pinpoint, telemetry))
        val ticker = sch.add(compose {
            onTick {
                initLog()
                false
            }
        })
        aprilTag = AprilTag(if (red) hw.webcam2 else hw.webcam1)
        sch.add(aprilTag!!.setupAprilTag(0, 0)).then(startFlag)

        shooter = sch.add(ShooterImpl(hw))

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

        startFlag.then(OneShot {
            pinpointSetupTask?.stop()
            ticker.stop()
        })

        startFlag.then(VirtualGroup {
            add(REmover.drive2Pose2(hw, poseSet.farShoot))
            add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.3))
        })
            .then(Combo.shoot(hw, shooter, 0.5))
            .then(VirtualGroup {
                val intake = add(Combo.shootAfter(hw)).then(Combo.intake(hw, 1.0))
                add(REmover.drive2Pose2(hw, poseSet.set4pos))
                    .then(REmover.drive2Pose2(hw, poseSet.set4out, 0.35))
                    .then(Wait.s(.5))
                    .then(OneShot {
                        intake.finish()
                    })
                    .then(VirtualGroup {
                        add(REmover.drive2Pose2(hw, poseSet.farShoot))
                        add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.2))
                    })
            })

            //shoot #2
            .then(Combo.shoot(hw, shooter, 0.5))
            .then(VirtualGroup {
                val intake = add(Combo.shootAfter(hw)).then(Combo.intake(hw, 1.0, timeout = 5.0))
                add(REmover.drive2Pose2(hw, poseSet.overflowPos1, timeoutAt = 0.5))
                    .then(REmover.drive2Pose2(hw, poseSet.overflowPos2, timeoutAt = 0.5))
//                    .then(REmover.drive2Pose2(hw, poseSet.set4out, 0.35))
                    .then(OneShot {
                        intake.finish()
                    })
                    .then(VirtualGroup {
                        add(REmover.drive2Pose2(hw, poseSet.farShoot))
                        add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.2))
                    })
            })
            .then(Combo.shoot(hw, shooter, 0.5))

            //shoot #3
            .then(VirtualGroup {
                val intake = add(Combo.shootAfter(hw)).then(Combo.intake(hw, 1.0, timeout = 5.0))
                add(REmover.drive2Pose2(hw, poseSet.overflowPos1, timeoutAt = 0.5))
                    .then(REmover.drive2Pose2(hw, poseSet.overflowPos2, timeoutAt = 0.5))
//                    .then(REmover.drive2Pose2(hw, poseSet.set4out, 0.35))
                    .then(OneShot {
                        intake.finish()
                    })
                    .then(VirtualGroup {
                        add(REmover.drive2Pose2(hw, poseSet.farShoot))
                        add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.2))
                    })
            })
            .then(Combo.shoot(hw, shooter, 0.5))
            .then(VirtualGroup {
                add(Combo.shootAfter(hw))
                add(REmover.drive2Pose2(hw, poseSet.auto2park))
            })

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
                reconfigure(StaticStore.prismBroken, scheduler!!)
            }
            if (x && !xt) {
                reconfigure(!StaticStore.prismBroken, scheduler!!)
            }

            rbt = rb
            xt = x

            return false // use finish() to kill this
        }
    }
}