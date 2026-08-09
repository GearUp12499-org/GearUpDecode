package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.variants.TurretImpl2
import android.util.Log
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.HOOD_50


class AimingMachine (private val hw: CompBot2Hardware, private val poseSet: PoseSet) {

    /*
    Things that belong to this class
    - shooter motors
    - turret servos
    - hood

    sensor inputs that this class needs:
    - tx (for ll only)
    - kalman position (for full)
    - pinpoint x and y velocities (for full)
     */

    enum class State {
        HARDCODE,
        LIMELIGHT_ONLY,
        FULL
    }

    var state = State.FULL
    private var prevState = State.FULL

    var turretPreset = 0.0
    var hoodPreset = HOOD_50
    var shooterPreset = 1800.0

    private val shooter = ShooterImpl2(hw)
    private val turret = TurretImpl2(hw)

    private var lastGoodReadTime = System.nanoTime()

    fun init() {
        shooter.init()

        state = State.FULL
        onEnter(State.FULL)
    }


//    changeModes: Boolean, deltaTarget: Double, lldistance: Double?,
//    robotVelX: Double, robotVelY: Double, goalPose: REmover.RobotPose,
//    robotPose: REmover.RobotPose, red: Boolean

    //deltaTarget = -target.targetXDegrees (see turretTrack LegacyTurretTrack)
    //lldistance = (taToDisstance(target.targetArea)- 10.0) see Legacy TurretTrack
    fun update(robotState: RobotState, input: GamepadState) {
        if (prevState != state){
            onEnter(state)
            Log.i("AimingMachine","TRANSITIONED TO $state")
        }
        prevState = state

        turret.tickTurret()
        shooter.tickShooter(robotState.shoot1Vel)

        if (input.x2) {
            when (state){
                State.HARDCODE -> transitionTo(State.FULL)
                State.LIMELIGHT_ONLY -> transitionTo(State.HARDCODE)
                State.FULL -> transitionTo(State.LIMELIGHT_ONLY)
            }
            return
        }
        when (state) {
            State.HARDCODE -> {
                if (turret.targetAngleDeg != turretPreset){
                    turret.targetAngleDeg = turretPreset
                }
                if (shooter.target != shooterPreset){
                    shooter.target = shooterPreset
                }
                if (hw.hood.position != hoodPreset){
                    hw.hood.position = hoodPreset
                }
            }

            State.LIMELIGHT_ONLY -> {
                val now = System.nanoTime()
                if (robotState.llDistance == null || robotState.tx == null){
                    if (now - lastGoodReadTime > 2e9){
                        turret.setTurretTarget(0.0)
                    }
                    return
                }
                turret.setTurretDeltaTarget(robotState.tx!!)
                val hoodSpeed = CompBot2Hardware.hoodAndSpeed(robotState.llDistance!!)
                hw.hood.position = hoodSpeed?.first ?: CompBot2Hardware.HOOD_50
                shooter.setTarget(hoodSpeed?.second ?: 1800.0)
                lastGoodReadTime = System.nanoTime()
            }

            State.FULL -> {
                val hoodSpeedTurret = CompBot2Hardware.hoodAndSpeedAndTurret(
                    robotState.velXM,
                    robotState.velYM,
                    poseSet.goalAT,
                    robotState.kalmanPose,
                    robotState.kalmanX < -24.0,
                    robotState.red
                )
                turret.setTurretTarget(hoodSpeedTurret?.third ?: 0.0)
                hw.hood.position = hoodSpeedTurret?.first ?: CompBot2Hardware.HOOD_50
                shooter.setTarget(hoodSpeedTurret?.second ?: 1800.0)
            }
        }
    }

    fun transitionTo(newState: State) {
        state = newState
    }

    fun onEnter(newState: State) {
        when (newState) {
            State.HARDCODE -> {
                turret.setTurretTarget(0.0)
                hw.hood.position = CompBot2Hardware.HOOD_50
                shooter.setTarget(1800.0)
            }

            else -> {
                return
            }
        }
    }
}

