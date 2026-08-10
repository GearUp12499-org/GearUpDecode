package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.HOOD_50
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOT_MID_RANGE
import org.firstinspires.ftc.teamcode.systems.REmover
import kotlin.math.hypot
import kotlin.math.max

abstract class StateAuto(private val red: Boolean): LinearOpMode() {

    lateinit var hw: CompBot2Hardware
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE

    enum class State {
        SHOOT,
        DRIVE_TO_MIDSHOOT_WAYPOINT,
        DRIVE_TO_MIDSHOOT,
        DRIVE_TO_SPIKE_1,
        PICKUP_SPIKE_1,
        DRIVE_TO_SPIKE_2,
        PICKUP_SPIKE_2,
        DRIVE_TO_SPIKE_3,
        PICKUP_SPIKE_3,
        DONE
    }

    private val sequence = listOf(
        State.DRIVE_TO_MIDSHOOT,
        State.SHOOT,
        State.DRIVE_TO_SPIKE_2,
        State.PICKUP_SPIKE_2,
        State.DRIVE_TO_MIDSHOOT_WAYPOINT,
        State.DRIVE_TO_MIDSHOOT,
        State.SHOOT,
        State.DRIVE_TO_SPIKE_1,
        State.PICKUP_SPIKE_1,
        State.DRIVE_TO_MIDSHOOT_WAYPOINT,
        State.DRIVE_TO_MIDSHOOT,
        State.SHOOT,
        State.DRIVE_TO_SPIKE_3,
        State.PICKUP_SPIKE_3,
//        State.DRIVE_TO_MIDSHOOT_WAYPOINT,
        State.DRIVE_TO_MIDSHOOT,
        State.SHOOT,
        State.DONE
        )

    var currentIndex = 0

    private var state = sequence[currentIndex]
    var prevState = state

    var justTransitioned = true

    lateinit var robotState: RobotState


    override fun runOpMode() {

        hw = CompBot2Hardware(hardwareMap)

        robotState = RobotState(hw, red)
        val gamepadState = GamepadState(gamepad1, gamepad2)


        val driveMachine = DriveMachine(hw)
        val intakeShootMachine = IntakeShootMachine(hw)
        val aimingMachine = AimingMachine(hw, poseSet)


        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        driveMachine.init(robotState)
        intakeShootMachine.init()
        aimingMachine.init()
        robotState.init()

        while (opModeInInit()){
            robotState.updateState()
        }

        waitForStart()

        aimingMachine.state = AimingMachine.State.HARDCODE

        aimingMachine.turretPreset = 0.0
        aimingMachine.hoodPreset = HOOD_50
        aimingMachine.shooterPreset = SHOOT_MID_RANGE

        driveMachine.state = DriveMachine.State.AUTO

        driveMachine.removerImpl.init(
            robotState.ppX,
            robotState.ppY,
            robotState.ppThetaRad,
            poseSet.midShoot
        )

        while (opModeIsActive()) {

            val start = System.nanoTime()

            telemetry.addData("Auto State", state)
            telemetry.addData("IntakeShoot", intakeShootMachine.state)
            telemetry.addData("Drive", driveMachine.state)
            telemetry.addData("Aiming", aimingMachine.state)

            Log.i("AutoState", state.toString())
            Log.i("IntakeShoot", intakeShootMachine.state.toString())
            Log.i("Drive", driveMachine.state.toString())
            Log.i("aiming", aimingMachine.state.toString())

            telemetry.addData("trying to drive to", driveMachine.removerImpl.target.x)
            telemetry.addData("trying to drive to", driveMachine.removerImpl.target.y)
            telemetry.addData("trying to drive to", driveMachine.removerImpl.target.a)

            Log.i("ppx", robotState.ppX.toString())
            Log.i("ppy", robotState.ppY.toString())
            Log.i("ppTheta", robotState.ppThetaRad.toString())


            telemetry.update()
            for (module in lynxModules) {
                module.clearBulkCache()
            }
            robotState.updateState()
            gamepadState.update()

            justTransitioned = state != prevState
            prevState = state

            when (state) {
                State.DRIVE_TO_MIDSHOOT_WAYPOINT -> {
                    if (justTransitioned){
                        if (from(poseSet.set1out)) {
                            driveMachine.removerImpl.finished = true
                        } else if (from(poseSet.set2out)){
                            driveMachine.removerImpl.init(
                                robotState.ppX,
                                robotState.ppY,
                                robotState.ppThetaRad,
                                poseSet.set2exit,
                                stopCond = StopConditions.Waypoint
                                )
                        } else if (from(poseSet.set3out)){
                            driveMachine.removerImpl.init(
                                robotState.ppX,
                                robotState.ppY,
                                robotState.ppThetaRad,
                                poseSet.set3curve,
                                stopCond = StopConditions.Waypoint
                            )
                        } else if (from(poseSet.goalStart)){
                            driveMachine.removerImpl.finished = true
                        } else if (from(poseSet.farStart)){
                            driveMachine.removerImpl.finished = true
                        } else{
                            driveMachine.removerImpl.finished = true
                        }
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.DRIVE_TO_MIDSHOOT -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppX,
                            robotState.ppY,
                            robotState.ppThetaRad,
                            poseSet.midShoot,
                            farStrafe = true
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.DRIVE_TO_SPIKE_1 -> {
                    if (justTransitioned){
                        intakeShootMachine.state = IntakeShootMachine.State.INTAKING_DISTANCE_SENSORS

                        driveMachine.removerImpl.init(
                            robotState.ppX,
                            robotState.ppY,
                            robotState.ppThetaRad,
                            poseSet.set1pos,
                            stopCond = StopConditions.Waypoint
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }else if (intakeShootMachine.state == IntakeShootMachine.State.OFF){
                        driveMachine.removerImpl.finished = true
                        nextState()
                    }
                }
                State.PICKUP_SPIKE_1 -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppX,
                            robotState.ppY,
                            robotState.ppThetaRad,
                            poseSet.set1out,
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        intakeShootMachine.state = IntakeShootMachine.State.FINISHING
                        nextState()
                    }else if (intakeShootMachine.state == IntakeShootMachine.State.OFF){
                        driveMachine.removerImpl.finished = true
                        nextState()
                    }
                }
                State.DRIVE_TO_SPIKE_2 -> {
                    if (justTransitioned){
                        intakeShootMachine.state = IntakeShootMachine.State.INTAKING_DISTANCE_SENSORS

                        driveMachine.removerImpl.init(
                            robotState.ppX,
                            robotState.ppY,
                            robotState.ppThetaRad,
                            poseSet.set2pos,
                            stopCond = StopConditions.Waypoint
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                        Log.i("1","")
                    } else if (intakeShootMachine.state == IntakeShootMachine.State.OFF){
                        driveMachine.removerImpl.finished = true
                        nextState()
                        Log.i("2","")
                    }
                }
                State.PICKUP_SPIKE_2 -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppX,
                            robotState.ppY,
                            robotState.ppThetaRad,
                            poseSet.set2out,
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        intakeShootMachine.state = IntakeShootMachine.State.FINISHING
                        nextState()
                    }else if (intakeShootMachine.state == IntakeShootMachine.State.OFF){
                        driveMachine.removerImpl.finished = true
                        nextState()
                    }
                }
                State.DRIVE_TO_SPIKE_3 -> {
                    if (justTransitioned){
                        intakeShootMachine.state = IntakeShootMachine.State.INTAKING_DISTANCE_SENSORS

                        driveMachine.removerImpl.init(
                            robotState.ppX,
                            robotState.ppY,
                            robotState.ppThetaRad,
                            poseSet.set3pos,
                            stopCond = StopConditions.Waypoint
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }else if (intakeShootMachine.state == IntakeShootMachine.State.OFF){
                        driveMachine.removerImpl.finished = true
                        nextState()
                    }
                }
                State.PICKUP_SPIKE_3 -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppX,
                            robotState.ppY,
                            robotState.ppThetaRad,
                            poseSet.set3out
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        intakeShootMachine.state = IntakeShootMachine.State.FINISHING
                        nextState()
                    } else if (intakeShootMachine.state == IntakeShootMachine.State.OFF){
                        driveMachine.removerImpl.finished = true
                        nextState()
                    }
                }
                State.SHOOT -> {
                    if (justTransitioned) {
                        intakeShootMachine.state = IntakeShootMachine.State.SHOOTING_PREFLIPPER
                    }
                    if (intakeShootMachine.state == IntakeShootMachine.State.OFF) {
                        nextState()
                    }
                }
                State.DONE -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.finished = true
                    }
                }
            }
            driveMachine.update(robotState, gamepadState)
            intakeShootMachine.update(robotState, gamepadState)
            aimingMachine.update(robotState, gamepadState)

            val end = System.nanoTime()
            val time = (start-end)/1e6

            Log.i("LoopTime", time.toString())
        }

    }
    fun from(point: REmover.RobotPose): Boolean {
        val dx = robotState.ppX - point.x
        val dy = robotState.ppY - point.y

        val dist = hypot(dx, dy)

        return (dist <= 5.0)
    }

    fun nextState(){
        currentIndex++
        state = sequence.getOrElse(currentIndex) {State.DONE}
    }
}