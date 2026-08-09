package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.BOTTOM_STOP_STOWED
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_UP
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_UP
import org.firstinspires.ftc.teamcode.utilities.StaticStore
import android.util.Log


class IntakeShootMachine(private val hw: CompBot2Hardware) {

    /*
    things that belong to this class
    - shooter ball stop
    - flipper
    - bottom ball stop
    - intake

    sensor inputs that this class needs:
    gp1rb (transition to STARTING)
    gp1lb (transition to OFF)
    gp2y (shoot)
    colorTopRight (distance)
    colorTopLeft (distance)
    colorBottomRight (distance)
    colorBottomLeft (distance)
    ramps (front, middle) booleans

     */

    enum class State {
        OFF,
        INTAKING_DISTANCE_SENSORS,
        INTAKING_RAMPS,
        FINISHING,
        SHOOTING_PREFLIPPER,
        SHOOTING_FLIPPER,
        RECOVER_FLIPPER_SHOOTERSTOP
    }

    lateinit var state: State
    private var prevState = State.OFF

    private var waitingFlipper = false

    companion object {
        private val INTAKE_POWER = 1.0
        private val OUTTAKE_POWER = CompBot2Hardware.OUTTAKE_POWER
    }

    private var conditionContiniousStartTimeNs: Long = 0L
    private var stateStartTimeNs: Long = 0L

    fun init() {
        state = State.RECOVER_FLIPPER_SHOOTERSTOP

        hw.bottomBallStop.position = BOTTOM_STOP_STOWED

    }

    fun update(robotState: RobotState, input: GamepadState) {

        val justTransitioned = (state != prevState)
        if (justTransitioned){
            Log.i("IntakeShootMachine","TRANSITIONED TO $state")
            clearTimers()
        }
        prevState = state

        if (input.lb1) {
            state = State.RECOVER_FLIPPER_SHOOTERSTOP
            return
        }

        if (input.y2 && state == State.OFF) {
            state = State.SHOOTING_PREFLIPPER
            return
        }

        when (state) {
            State.RECOVER_FLIPPER_SHOOTERSTOP -> {
                if (justTransitioned) {
                    if (hw.shooterBallStop.position != SHOOTER_STOP_DOWN) {
                        hw.shooterBallStop.position = SHOOTER_STOP_DOWN
                    }
                    waitingFlipper = (hw.flipper.position != FLIPPER_DOWN)
                    if (waitingFlipper) {
                        hw.setIntakePower(OUTTAKE_POWER)
                        hw.flipper.position = FLIPPER_DOWN
                    } else{
                        state = State.OFF
                        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
                        return
                    }
                }
                if (getElapsedSec(stateStartTimeNs) > 0.5) {
                    state = State.OFF
                    hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
                }
            }

            State.OFF -> {
                if (justTransitioned) {
                    hw.setIntakePower(0.0)
                    hw.shooterBallStop.position = SHOOTER_STOP_UP
                }
                if (input.rb1) {
                    state = State.INTAKING_DISTANCE_SENSORS
                }
            }

            State.INTAKING_DISTANCE_SENSORS -> {
                if (justTransitioned) {
                    hw.shooterBallStop.position = SHOOTER_STOP_DOWN
                    hw.setIntakePower(INTAKE_POWER)
                    hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2)
                }
                if (robotState.colorTopLeft < 95.0 || robotState.colorTopRight < 95.0) {
                    state = State.INTAKING_RAMPS
                    conditionContiniousStartTimeNs = 0L
                }
            }

            State.INTAKING_RAMPS -> {
                if (robotState.frontRamp && robotState.middleRamp) {
                    if (conditionContiniousStartTimeNs == 0L) {
                        conditionContiniousStartTimeNs = System.nanoTime()
                    } else if (getElapsedSec(conditionContiniousStartTimeNs) >= 0.5) {
                        state = State.FINISHING
                        return
                    }
                } else {
                    conditionContiniousStartTimeNs = 0L
                }
            }

            State.FINISHING -> {
                if (justTransitioned) {
                    hw.setIntakePower(0.0)
                    hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3)
                }
                if (getElapsedSec(stateStartTimeNs) >= 0.15) {
                    state = State.RECOVER_FLIPPER_SHOOTERSTOP
                }
            }

            State.SHOOTING_PREFLIPPER -> {
                if (justTransitioned) {
                    hw.setIntakePower(INTAKE_POWER)
                    hw.shooterBallStop.position = SHOOTER_STOP_UP
                    hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4)
                }
                val stuck = !robotState.frontRamp && (robotState.colorBottomLeft < 110.0
                        || robotState.colorBottomRight < 110.0)
                if (getElapsedSec(stateStartTimeNs) > 1.0) {
                    hw.flipper.position = FLIPPER_UP
                    state = State.SHOOTING_FLIPPER
                    return
                }
                if (stuck) {
                    if (conditionContiniousStartTimeNs == 0L) {
                        conditionContiniousStartTimeNs = System.nanoTime()
                    } else if (getElapsedSec(conditionContiniousStartTimeNs) > 0.15) {
                        hw.flipper.position = FLIPPER_UP
                        state = State.SHOOTING_FLIPPER
                        return
                    }
                } else {
                    conditionContiniousStartTimeNs = 0L
                }
            }

            State.SHOOTING_FLIPPER -> {
                if (getElapsedSec(stateStartTimeNs) > 0.4) {
                    state = State.RECOVER_FLIPPER_SHOOTERSTOP
                }
            }


        }
    }


private fun getElapsedSec(startTimeNs: Long): Double {
    return (System.nanoTime() - startTimeNs) / 1_000_000_000.0
}

private fun clearTimers() {
    val now = System.nanoTime()
    conditionContiniousStartTimeNs = 0L
    stateStartTimeNs = now
}
}