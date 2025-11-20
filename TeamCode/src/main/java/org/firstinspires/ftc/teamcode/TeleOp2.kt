package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.TeleOpOptions.DRIVE_PUSH_TO_OVERRIDE
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.Locks
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.systems.Indexer
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.In1
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.In2
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.In3
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.Out1
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.Out2
import org.firstinspires.ftc.teamcode.systems.Indexer.Position.Out3
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.Shooter
import org.firstinspires.ftc.teamcode.systems.shootThree
import org.firstinspires.ftc.teamcode.tasks.PinpointUpdater
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.tasks.stopAllWith
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.time.DurationUnit

abstract class TeleOp2 : LinearOpMode() {
    abstract val negateIfRed: Int
    abstract val negateIfBlue: Int

    lateinit var hardware: CompBotHardware
    lateinit var scheduler: Scheduler
    lateinit var indexer: Indexer
    lateinit var shooter: Shooter
    lateinit var startFlag: SentinelTask

    private var fieldCentricDrivePOV = 0.0

    private fun setupPerAllianceValues() {
        fieldCentricDrivePOV = PI / 2 * negateIfBlue
    }

    override fun runOpMode() {
        TaskSharkAndroid.setup()
        hardware = CompBotHardware(hardwareMap)
        scheduler = FastScheduler()
        startFlag = scheduler.add(SentinelTask())

        // Setup
        setupPerAllianceValues()
        with(hardware) {
            val interDuration = Lifetime.timeSinceLastActive()
            if (interDuration.toDouble(DurationUnit.SECONDS) > 15.0) {
                Log.i("TeleOp2", "recalibrating, $interDuration since last execute")
                pinpoint.resetPosAndIMU()
            }

            scheduler.add(PinpointUpdater(pinpoint))

            val waitForPinpointReady =
                scheduler.add(WaitUntil {
                    pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY
                })
            waitForPinpointReady.then(startFlag)

            this@TeleOp2.indexer = scheduler.add(
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
            this@TeleOp2.shooter = scheduler.add(
                Shooter(
                    motor = shooter1,
                    indicator1 = indicator1,
                    indicator2 = indicator2
                )
            )

            flipper.position = CompBotHardware.FLIPPER_DOWN
        }

        startFlag.then(indexer.syncPosition())

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
        startFlag.requestStart()
        scheduler.tick()

        // holding...
        while (opModeIsActive() && startFlag.getState() != ITask.State.Finished) {
            scheduler.tick()
            telemetry.addLine("== ROBOT INIT IS BUSY ==")
            telemetry.addLine("Press [1:Back] to interrupt")
            telemetry.addLine()
            if (gamepad1.back) {
                startFlag.finish()
                continue
            }
            val tasks = startFlag.dependedTasks()
            val completed = tasks.count { it.getState() == ITask.State.Finished }
            telemetry.addData("Prerequisites", "$completed/${tasks.size}")
            for (task in tasks.filter { it.getState() != ITask.State.Finished }) {
                telemetry.addLine("${task.describeVerbose()} ... ${task.getState()}")
            }
            telemetry.update()
        }

        // RUNNING
        while (opModeIsActive()) {
            scheduler.tick()
            driverInput()
            telemetry.update()
        }
    }

    // Inputs

    var wasA = false
    var wasB = false
    var wasY = false
    var wasY2 = false

    private fun dispColor(label: String, sensor: RevColorSensorV3) {
        val norm = sensor.normalizedColors
        telemetry.addData(
            label, "%.2fmm rgb %.2f %.2f %.2f a %.2f".format(
                sensor.getDistance(DistanceUnit.MM),
                norm.red,
                norm.green,
                norm.blue,
                norm.alpha,
            )
        )
    }

    private fun getNextOut(pos: Indexer.Position) = when (pos) {
        Out1 -> Out2
        Out2 -> Out3
        Out3 -> Out1
        else -> Out1
    }

    private fun getNextIn(pos: Indexer.Position) = when (pos) {
        In1 -> In2
        In2 -> In3
        In3 -> In1
        else -> In1
    }

    fun driverInput() {
        driveInputs()
        if (scheduler.getLockOwner(Locks.DRIVE_MOTORS) == null) {
            drive()
        } else if (driveInputIsOverriding()) {
            scheduler.stopAllWith(Locks.DRIVE_MOTORS)
            drive()
        }

        intake()
        recoveryIntake()
        resetOrientation()
        rehome()

//        dispColor("fc1", hardware.frontColor1)
//        dispColor("fc2", hardware.frontColor2)
//        dispColor("bc1", hardware.backColor1)
//        dispColor("bc2", hardware.backColor2)
//
//        telemetry.addData("idx1", hardware.idxMag1.state)
//        telemetry.addData("idx2", hardware.idxMag2.state)
//        telemetry.addData("idx3", hardware.idxMag3.state)
//        telemetry.addData("idx4", hardware.idxMag4.state)
        telemetry.addData("slot1", indexer.slots[0])
        telemetry.addData("slot2", indexer.slots[1])
        telemetry.addData("slot3", indexer.slots[2])
//        telemetry.addData("ipos", indexer.lastPosition)
//        telemetry.addData("ipos", hardware.indexer.currentPosition)

        val a = gamepad1.a
        val b = gamepad1.b
        if (a && !wasA) {
            scheduler.stopAllWith(indexer.lock)
            scheduler.add(indexer.goToPosition(getNextOut(indexer.lastPosition)))
        }
        if (b && !wasB) {
            scheduler.stopAllWith(indexer.lock)
            scheduler.add(indexer.goToPosition(getNextIn(indexer.lastPosition)))
        }

        val y = gamepad1.y
        if (y && !wasY) {
            scheduler.stopAllWith(indexer.lock)
            scheduler.add(VirtualGroup {
                add(REmover.drive2Pose(hardware, CompBotHardware.shootPos))
                add(OneShot {
                    shooter.setTarget(1200.0)
                })
            }).then(shootThree(1200.0, shooter, indexer))
        }

        val y2 = gamepad2.y
        if (y2 && !wasY2) {
            scheduler.stopAllWith(indexer.lock)
            scheduler.add(shootThree(1200.0, shooter, indexer))
        }

        wasA = a
        wasB = b
        wasY = y
    }

    var gp1lStickX = 0.0f
    var gp1lStickY = 0.0f
    var gp1rStickX = 0.0f

    fun driveInputs() {
        gp1lStickX = gamepad1.left_stick_x
        gp1lStickY = gamepad1.left_stick_y
        gp1rStickX = gamepad1.right_stick_x
    }

    fun driveInputIsOverriding() =
        hypot(gp1lStickX, gp1lStickY) > DRIVE_PUSH_TO_OVERRIDE
                || abs(gp1rStickX) > DRIVE_PUSH_TO_OVERRIDE

    fun drive() {
        val y = -gp1lStickY
        val x = gp1lStickX
        val rx = gp1rStickX

        val heading = hardware.pinpoint.getHeading(AngleUnit.RADIANS) + fieldCentricDrivePOV
        var strafe = x * cos(-heading) - y * sin(-heading)
        val forward = x * sin(-heading) + y * cos(-heading)

        // gm0 says
        strafe *= 1.1

        val denominator = max(abs(forward) + abs(strafe) + abs(rx), 1.0)

        val postFactor =
            if (gamepad1.left_trigger > TeleOpOptions.SLOW_BUTTON_SENSITIVITY) TeleOpOptions.SLOW_BUTTON_MULTIPLIER
            else 1.0

        val finalDiv = denominator / postFactor

        val flP = (forward + strafe + rx) / finalDiv
        val blP = (forward - strafe + rx) / finalDiv
        val frP = (forward - strafe - rx) / finalDiv
        val brP = (forward + strafe - rx) / finalDiv

        hardware.frontLeft.power = flP
        hardware.frontRight.power = frP
        hardware.backLeft.power = blP
        hardware.backRight.power = brP
    }

    private var intakeTask: ITask<*>? = null
    private var wasEnable = false
    private var wasCancel = false

    fun intake() {
        val enableBtn = gamepad1.right_bumper
        val cancelBtn = gamepad1.left_bumper

        val enable = enableBtn && !wasEnable
        val cancel = cancelBtn && !wasCancel
        if (enable || cancel) {
            shooter.stopSoft()
            scheduler.stopAllWith(indexer.lock)
            intakeTask?.let {
                if (it.getState() == ITask.State.Ticking) it.stop()
            }
            intakeTask = if (enable) scheduler.add(indexer.intake()) else null
        }

        wasEnable = enableBtn
        wasCancel = cancelBtn
    }

    var wasUsingManualCtrl = false

    fun recoveryIntake() {
        val button1 = gamepad2.right_bumper
        val button2 = gamepad2.left_bumper
        if (button1 || button2) {
            scheduler.stopAllWith(Locks.INTAKE)

            hardware.intake.power = if (button1) 1.0 else -1.0
            wasUsingManualCtrl = true
        } else if (wasUsingManualCtrl) {
            wasUsingManualCtrl = false
            hardware.intake.power = 0.0
        }
    }

    private var wasResetOrientation = false
    private var wasResetIdx = false

    fun resetOrientation() {
        val button = gamepad2.back
        if (button && !wasResetOrientation) {
            scheduler.stopAllWith(Locks.DRIVE_MOTORS)
            scheduler.add(VirtualGroup {
                val flag = add(OneShot {
                    hardware.frontLeft.power = 0.0
                    hardware.frontRight.power = 0.0
                    hardware.backLeft.power = 0.0
                    hardware.backRight.power = 0.0
                    hardware.indicator1.position = 0.4
                    hardware.indicator2.position = 0.4
                }).then(WaitUntil {
                    val vel = hypot(
                        hardware.pinpoint.getVelX(DistanceUnit.INCH),
                        hardware.pinpoint.getVelY(DistanceUnit.INCH)
                    )
                    vel < 0.5
                }).then(OneShot {
                    hardware.pinpoint.recalibrateIMU()
                }).then(WaitUntil {
                    hardware.pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY
                }).then(OneShot {
                    hardware.indicator1.position = 0.0
                    hardware.indicator2.position = 0.0
                })
            }).require(Locks.DRIVE_MOTORS)
        }
        wasResetOrientation = button
    }

    fun rehome() {
        val button = gamepad1.back
        if (button && !wasResetIdx) {
            scheduler.stopAllWith(indexer.lock)
            scheduler.add(VirtualGroup {
                add(OneShot {
                    hardware.indicator1.position = 0.37
                    hardware.indicator2.position = 0.37
                }).then(
                    indexer.syncPosition()
                ).then(OneShot {
                    hardware.indicator1.position = 0.0
                    hardware.indicator2.position = 0.0
                })
            })
        }
        wasResetIdx = button
    }
}