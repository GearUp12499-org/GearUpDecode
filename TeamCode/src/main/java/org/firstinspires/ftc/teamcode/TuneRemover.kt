package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.Locks
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_MID_RANGE
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.tasks.PinpointSetupTask
import org.firstinspires.ftc.teamcode.tasks.PinpointTask
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.tasks.compose
import org.firstinspires.ftc.teamcode.tasks.stopUsing
import org.firstinspires.ftc.teamcode.utilities.StaticStore

@TeleOp
class TuneRemover : LinearOpMode() {

    private lateinit var hw: CompBot2Hardware

    private var pinpointSetupTask: PinpointSetupTask? = null

    private lateinit var shooter: ShooterImpl

    private val poseSet = PoseSet.RED

    fun initLog() {

        pinpointSetupTask?.let {
            telemetry.addLine()
            telemetry.addData("Linear velo (in/s)", it.velocity)
            telemetry.addData("Angular velo (rad/s)", it.angularVelocity)
        }
        telemetry.update()
    }


    override fun runOpMode() {
        TaskSharkAndroid.setup()
        val sch = FastScheduler()
        val startFlag = sch.add(SentinelTask())
        hw = CompBot2Hardware(hardwareMap)

        sch.add(Group {
            sch.add(OneShot {
                hw.pinpoint.recalibrateIMU()
            }).then(WaitUntil {
                hw.pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY
            }).then(OneShot {
                hw.pinpoint.setPosition(REmover.RobotPose(0.0,0.0,0.0).asPose2D)
            })

        })

        pinpointSetupTask = sch.add(PinpointSetupTask(hw.pinpoint, telemetry))
        val ticker = sch.add(compose {
            onTick {
                initLog()
                false
            }
        })

        sch.add(compose {
            var state: GoBildaPinpoint2Driver.DeviceStatus? = null
            onTick {
                val stateNew = hw.pinpoint.deviceStatus
                if (stateNew != state) {
                    Log.w("Pinpoint", "state changed $state -> $stateNew")
                    state = stateNew
                }
                false
            }
        })

        sch.add(PinpointTask(hw.pinpoint))
        shooter = startFlag.then(ShooterImpl(hw))

        startFlag.then(OneShot {
            pinpointSetupTask?.stop()
            ticker.stop()
        })
        startFlag.then(DriveTask())

        while (opModeInInit()) sch.tick()

        startFlag.requestStart()

        while (opModeIsActive()) {
            sch.tick()
            hw.pinpoint.update()

        }
    }

    private inner class DriveTask : Task<DriveTask>() {
        private inline val sch2 : Scheduler get() = super.scheduler!!

        var wasA = false
        var wasB = false
        var wasX = false
        var wasY = false

        var wasBack = false

        override fun onTick(): Boolean {
            val sch2 = sch2

            if (gamepad1.a && !wasA) {
                sch2.stopUsing(Locks.DRIVE_MOTORS)
                sch2.add(REmover.drive2Pose2(hw, REmover.RobotPose(5.0, 0.0, 0.0)))
            }
            else if (gamepad1.b && !wasB) {
                sch2.stopUsing(Locks.DRIVE_MOTORS)
                sch2.add(REmover.drive2Pose2(hw, poseSet.set2pos))
                sch2.add(REmover.drive2Pose2(hw, poseSet.set2out))
            }
            else if (gamepad1.x && !wasX) {
                sch2.stopUsing(Locks.DRIVE_MOTORS)
                sch2.add(REmover.drive2Pose2(hw, REmover.RobotPose(-12.0,-24.0,-Math.PI/2), waypoint = true, minPower = 1.0))
                sch2.add(REmover.drive2Pose2(hw, poseSet.set2out))

            }
            else if (gamepad1.y && !wasY) {
                sch2.stopUsing(Locks.DRIVE_MOTORS)
                sch2.add(REmover.drive2Pose2(hw, poseSet.midShoot))
            }
            else if (gamepad1.start) {
                sch2.stopUsing(Locks.DRIVE_MOTORS)
                sch2.add(REmover.drive2Pose2(hw, REmover.RobotPose(0.0,0.0,0.0)))
            }
            if (gamepad1.back && !wasBack) {
                sch2.add(shooter.setTargetAsync(SHOOT_MID_RANGE))
            }

            wasA = gamepad1.a
            wasB = gamepad1.b
            wasX = gamepad1.x
            wasY = gamepad1.y
            wasBack = gamepad1.back

            return false
        }
    }
}
