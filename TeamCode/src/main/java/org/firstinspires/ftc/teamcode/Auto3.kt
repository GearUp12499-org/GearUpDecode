package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.GSC_EXPOSURE
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.GSC_GAIN
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.SHOOT_CLOSE_RANGE
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.systems.AprilTag
import org.firstinspires.ftc.teamcode.systems.Indexer
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.Shooter
import org.firstinspires.ftc.teamcode.systems.shootThree
import org.firstinspires.ftc.teamcode.tasks.PinpointUpdater
import org.firstinspires.ftc.teamcode.tasks.SentinelTask

abstract class Auto3(isRed: Boolean) : LinearOpMode() {
    companion object {
        val obeliskToIndexer = mapOf(
            AprilTag.Obelisk.GPP to Indexer.Position.Out3,
            AprilTag.Obelisk.PGP to Indexer.Position.Out2,
            AprilTag.Obelisk.PPG to Indexer.Position.Out1,
        )
    }

    val poseSet = if (isRed) PoseSet.RED else PoseSet.BLUE

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
            pinpoint.position = poseSet.goalStart.asPose2D
            pinpoint.recalibrateIMU()

            scheduler.add(PinpointUpdater(pinpoint))

            val waitForPinpointReady =
                scheduler.add(WaitUntil {
                    pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY
                })
            waitForPinpointReady.then(startFlag)

            this@Auto3.indexer = scheduler.add(
                Indexer(
                    indexerMotor = indexer,
                    flipper = flipper,
                    intakeMotor = intake,
                    sensor1 = idxMag1,
                    sensor2 = idxMag2,
                    sensor3 = idxMag3,
                    sensor4 = idxMag4,
                    colorFront1 = frontColor1,
                    colorFront2 = frontColor2,
                    colorBack1 = backColor1,
                    colorBack2 = backColor2,
                    indicator1 = indicator1,
                    indicator2 = indicator2,
                )
            )

            this@Auto3.shooter = scheduler.add(
                Shooter(
                    motor = shooter1,
                    indicator1 = indicator1,
                    indicator2 = indicator2
                )
            )
            this@Auto3.aprilTag = AprilTag(gsc)
            val aprilTagSetup = scheduler.add(aprilTag.setupAprilTag(GSC_EXPOSURE, GSC_GAIN))
            aprilTagSetup.then(startFlag)
        }
        indexer.slots[0] = Indexer.Slot.PURPLE
        indexer.slots[1] = Indexer.Slot.PURPLE
        indexer.slots[2] = Indexer.Slot.GREEN


//        val knowObelisk = startFlag.then(aprilTag.readObelisk(1.0))
        val indexerReady = startFlag.then(indexer.syncPosition())
        startFlag.then(shooter.setTargetAndWait(SHOOT_CLOSE_RANGE))

        startFlag.then(VirtualGroup {
                add(REmover.drive2Pose(hardware, poseSet.closeShoot))
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
                    SHOOT_CLOSE_RANGE,
                    shooter,
                    indexer
                ) { aprilTag.obelisk?.let { obeliskToIndexer[it] } ?: Indexer.Position.Out1 }
            )

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