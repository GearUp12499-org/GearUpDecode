package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.ITask
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.TeleOpOptions.DRIVE_PUSH_TO_OVERRIDE
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver
import org.firstinspires.ftc.teamcode.tasks.DAEMON_TAGS
import org.firstinspires.ftc.teamcode.tasks.PinpointUpdater
import org.firstinspires.ftc.teamcode.tasks.SentinelTask
import org.firstinspires.ftc.teamcode.tasks.stopAllWith
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

@TeleOp(group = "!", name = "Teleop")
class TeleOp2 : LinearOpMode() {
    lateinit var hardware: CompBotHardware
    lateinit var scheduler: Scheduler
    val startFlag = SentinelTask()

    inner class ReportLockOwnershipTask : Task<ReportLockOwnershipTask>() {
        private val targets = listOf(
            CompBotHardware.Locks.DRIVE_MOTORS,
            CompBotHardware.Locks.INDEXER,
            CompBotHardware.Locks.INTAKE,
        )

        override fun getTags() = DAEMON_TAGS

        override fun onTick(): Boolean {
            for (lock in targets) {
                val own = scheduler?.getLockOwner(lock)
                telemetry.addData(lock.getFriendlyName(),
                    own?.describeVerbose() ?: "<free>"
                )
            }
            return false
        }
    }

    override fun runOpMode() {
        TaskSharkAndroid.setup()
        hardware = CompBotHardware(hardwareMap)
        scheduler = FastScheduler()
        scheduler.add(startFlag)

        // Setup
        with(hardware) {
            // TODO: remove [when we add auto]
            pinpoint.resetPosAndIMU()
//            pinpoint.recalibrateIMU()

            scheduler.add(PinpointUpdater(pinpoint))

            val waitForPinpointReady =
                scheduler.add(WaitUntil {
                    pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY
                })
            startFlag.require(waitForPinpointReady)
        }

        scheduler.add(ReportLockOwnershipTask())

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
            telemetry.addLine("== WAITING TO START... ==")
            telemetry.addLine("press Start on gamepad1 to override")
            telemetry.addLine()
            if (gamepad1.start) {
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

    fun driverInput() {
        driveInputs()
        if (scheduler.getLockOwner(CompBotHardware.Locks.DRIVE_MOTORS) == null) {
            drive()
        } else if (driveInputIsOverriding()) {
            scheduler.stopAllWith(CompBotHardware.Locks.DRIVE_MOTORS)
            drive()
        }
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
                || gp1rStickX > DRIVE_PUSH_TO_OVERRIDE

    fun drive() {
        val y = -gp1lStickY
        val x = gp1lStickX
        val rx = gp1rStickX

        // TODO: This should really be solved by setting the pinpoint's pose.
        // note: this call just reads a field. it does not go to hardware.
        val heading = hardware.pinpoint.getHeading(AngleUnit.RADIANS) + PI / 2
        var strafe = x * cos(-heading) - y * sin(-heading)
        val forward = x * sin(-heading) + y * cos(-heading)

        // gm0 says
        strafe *= 1.1

        val denominator = max(abs(forward) + abs(strafe) + abs(rx), 1.0)
        val flP = (forward + strafe + rx) / denominator
        val frP = (forward - strafe + rx) / denominator
        val blP = (forward - strafe - rx) / denominator
        val brP = (forward + strafe - rx) / denominator

        hardware.frontLeft.power = flP
        hardware.frontRight.power = frP
        hardware.backLeft.power = blP
        hardware.backRight.power = brP
    }
}