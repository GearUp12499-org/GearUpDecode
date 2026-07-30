package org.firstinspires.ftc.teamcode.variants

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.AimingMachine
import org.firstinspires.ftc.teamcode.DriveMachine
import org.firstinspires.ftc.teamcode.GamepadState
import org.firstinspires.ftc.teamcode.IntakeShootMachine
import org.firstinspires.ftc.teamcode.KalmanImpl
import org.firstinspires.ftc.teamcode.PoseSet
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.Kalman
import org.firstinspires.ftc.teamcode.systems.Kalman.Companion.INCHES_PER_METER
import org.firstinspires.ftc.teamcode.systems.Kalman.Companion.TICKS_PER_DEG
import org.firstinspires.ftc.teamcode.systems.TurretTrack.Companion.TAG_BLUE
import org.firstinspires.ftc.teamcode.systems.TurretTrack.Companion.TAG_RED
import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

abstract class StateTeleop (private val red: Boolean): LinearOpMode() {

    private lateinit var hw: CompBot2Hardware

    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE


    override fun runOpMode() {
        hw = CompBot2Hardware(hardwareMap)

        val driveMachine = DriveMachine(hw)
        val intakeShootMachine = IntakeShootMachine(hw)
        val aimingMachine = AimingMachine(hw, poseSet)

        val robotState = RobotState(hw, red)
        val gamepadState = GamepadState(gamepad1, gamepad2)

        hw.pinpoint.setPosition(Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0))

        waitForStart()

        driveMachine.init()
        intakeShootMachine.init()
        aimingMachine.init()
        robotState.init()

        var loopStartTime = System.nanoTime()
        val looptimeLimitMs = 100.0

        var goodLLRead = true

        while (opModeIsActive()) {

            //DO ALL SENSOR READS HERE
            robotState.updateState()
            //get the current state of the gamepad
            gamepadState.update()

            driveMachine.update(robotState, gamepadState)
            intakeShootMachine.update(robotState, gamepadState)
            aimingMachine.update(robotState, gamepadState)

            while (System.nanoTime() / 1_000_000 - loopStartTime < looptimeLimitMs) {
                sleep(5)
            }
            loopStartTime = System.nanoTime()

        }
    }
}


