package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.EXPOSURE
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.GAIN
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.SHOOT_MIDRANGE
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.redFarStart
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.set1pos
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.shootPos
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.systems.AprilTag
import org.firstinspires.ftc.teamcode.systems.Indexer
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.Shooter
import org.firstinspires.ftc.teamcode.systems.shootThree
import org.firstinspires.ftc.teamcode.tasks.PinpointUpdater
import org.firstinspires.ftc.teamcode.tasks.SentinelTask

@Autonomous
class Auto1 : LinearOpMode() {
    companion object {
        val obeliskToIndexer = mapOf(
            AprilTag.Obelisk.GPP to Indexer.Position.Out3,
            AprilTag.Obelisk.PGP to Indexer.Position.Out2,
            AprilTag.Obelisk.PPG to Indexer.Position.Out1,
        )
    }

    private lateinit var shooter: Shooter
    private lateinit var indexer: Indexer
    private lateinit var aprilTag: AprilTag
    private lateinit var startFlag: SentinelTask
    private lateinit var scheduler: FastScheduler
    private lateinit var hardware: CompBotHardware

    override fun runOpMode() {
        TaskSharkAndroid.setup()
        hardware = CompBotHardware(hardwareMap)
        scheduler = FastScheduler()
        startFlag = scheduler.add(SentinelTask())

        with(hardware) {
            pinpoint.position = redFarStart.asPose2D
            pinpoint.recalibrateIMU()

            scheduler.add(PinpointUpdater(pinpoint))

            val waitForPinpointReady =
                scheduler.add(WaitUntil {
                    pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY
                })
            waitForPinpointReady.then(startFlag)

            this@Auto1.indexer = scheduler.add(
                Indexer(
                    indexerMotor = indexer,
                    sensor1 = idxMag1,
                    sensor2 = idxMag2,
                    sensor3 = idxMag3,
                    sensor4 = idxMag4,
                    colorFront1 = hardware.frontColor1,
                    colorFront2 = hardware.frontColor2,
                    colorBack1 = hardware.backColor1,
                    colorBack2 = hardware.backColor2,
                )
            )

            this@Auto1.shooter = scheduler.add(
                Shooter(
                    motor = shooter1,
                    angle = shooterHood1
                )
            )
            this@Auto1.aprilTag = AprilTag(gsc)
            val aprilTagSetup = scheduler.add(aprilTag.setupAprilTag(EXPOSURE, GAIN))
            aprilTagSetup.then(startFlag)
        }
        indexer.slots[0] = Indexer.Slot.PURPLE
        indexer.slots[1] = Indexer.Slot.PURPLE
        indexer.slots[2] = Indexer.Slot.GREEN


        val knowObelisk = startFlag.then(aprilTag.readObelisk(1.0))
        val indexerReady = startFlag.then(indexer.syncPosition())
        startFlag.then(shooter.setTargetAndWait(SHOOT_MIDRANGE))

        knowObelisk
            .then(VirtualGroup {
                add(REmover.drive2Pose(hardware, shootPos))
                val idxMove = add(
                    indexer.goToPosition {
                        aprilTag.obelisk?.let { obeliskToIndexer[it] } ?: Indexer.Position.Out1
                    })
                add(OneShot {
                    Log.i("April Tag read", aprilTag.obelisk.toString())
                    Log.i(
                        "April Tag read",
                        (aprilTag.obelisk?.let { obeliskToIndexer[it] }
                            ?: Indexer.Position.Out1).toString()
                    )
                })
                indexerReady.then(idxMove)
            })
            .then(
                shootThree(
                    shooter,
                    indexer,
                    hardware.flipper
                ) { aprilTag.obelisk?.let { obeliskToIndexer[it] } ?: Indexer.Position.Out1 }
            )
            .then(REmover.drive2Pose(hardware, set1pos))

        // INIT
        while (opModeInInit()) {
            scheduler.tick()
            val tasks = startFlag.dependedTasks()
            val completed = tasks.count { it.getState() == ITask.State.Finished }
            telemetry.addData("Start Prereqs", "$completed/${tasks.size}")
            for (task in tasks.filter { it.getState() != ITask.State.Finished }) {
                telemetry.addLine("${task.describeVerbose()} ... ${task.getState()}")
            }
            telemetry.update()
        }

        // TRANSITION TO START
        startFlag.finish()
        scheduler.tick()

        Lifetime.bump()

        while (opModeIsActive()) {
            scheduler.tick()
            telemetry.update()
        }

        Lifetime.bump()
    }
}