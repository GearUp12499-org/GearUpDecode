package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.variants.TurretImpl2

class AimingMachine (private val hw: CompBot2Hardware) {

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

    public enum class State {
        HARDCODE,
        LIMELIGHT_ONLY,
        FULL
    }

    var state = State.FULL
        private set
    private var prevState = State.FULL

    private val shooter = ShooterImpl2(hw)
    private val turret = TurretImpl2(hw)

    fun init() {
        state = State.FULL
        onEnter(State.FULL)
    }

    //deltaTarget = -target.targetXDegrees (see turretTrack LegacyTurretTrack)
    //lldistance = (taToDisstance(target.targetArea)- 10.0) see Legacy TurretTrack
    fun update(
        changeModes: Boolean, deltaTarget: Double, lldistance: Double,
        robotVelX: Double, robotVelY: Double, goalPose: REmover.RobotPose,
        robotPose: REmover.RobotPose, red: Boolean
    ) {
        if (prevState != state){
            onEnter(state)
        }
        prevState = state

        turret.tickTurret()
        shooter.tickShooter()

        if (changeModes) {
            when (state){
                State.HARDCODE -> transitionTo(State.FULL)
                State.LIMELIGHT_ONLY -> transitionTo(State.HARDCODE)
                State.FULL -> transitionTo(State.LIMELIGHT_ONLY)
            }
            return
        }
        when (state) {
            State.HARDCODE -> {
                return
            }

            State.LIMELIGHT_ONLY -> {
                turret.setTurretDeltaTarget(deltaTarget)
                val hoodSpeed = CompBot2Hardware.hoodAndSpeed(lldistance)
                hw.hood.position = hoodSpeed?.first ?: CompBot2Hardware.HOOD_50
                shooter.setTarget(hoodSpeed?.second ?: 1800.0)
            }

            State.FULL -> {
                val hoodSpeedTurret = CompBot2Hardware.hoodAndSpeedAndTurret(
                    robotVelX,
                    robotVelY,
                    goalPose,
                    robotPose,
                    robotPose.x < -24.0,
                    red
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

