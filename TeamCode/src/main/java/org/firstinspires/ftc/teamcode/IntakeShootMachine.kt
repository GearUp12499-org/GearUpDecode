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

class IntakeShootMachine (private val hw: CompBot2Hardware){

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
    colorTopRight
    colorTopLeft
    ramps

     */

    public enum class State {
        OFF,
        STARTING,
        INTAKING,
        FINISHING,
        SHOOTING
    }

    var state = State.OFF
        private set
    private var prevState = State.OFF

    private enum class INTAKINGSubState{
        WAIT_FOR_DISTANCE_SENSORS,
        WAIT_FOR_RAMP_CONTINUOUS
    }
    private var intakeSubState = INTAKINGSubState.WAIT_FOR_DISTANCE_SENSORS

    private enum class FINISHINGSubState{
        INTAKE_SETTLING,
        FINISH_MOVES
    }

    private var finishingSubState = FINISHINGSubState.INTAKE_SETTLING

    private enum class SHOOTINGSubState{
        PRE_FLIPPER,
        FLIPPERING,
        POST_FLIPPER
    }

    private var shootingSubState = SHOOTINGSubState.PRE_FLIPPER

    companion object {
        private val INTAKE_POWER = 1.0
        private val OUTTAKE_POWER = CompBot2Hardware.OUTTAKE_POWER
    }

    private var stepStartTimeNs: Long = 0L
    private var conditionContiniousStartTimeNs: Long = 0L
    private var stateStartTimeNs: Long = 0L

    fun init(){
        state = State.OFF
        onStateEnter(State.OFF)
    }

    fun update(gp1rb: Boolean, gp1lb: Boolean, gp2y: Boolean,
               topLeftDistance: Double, topRightDistance: Double,
               rampsActive: Boolean){

        val justTransitioned = (state != prevState)
        prevState = state

        if (justTransitioned) {
            onStateEnter(state)
        }

        if (gp1lb){
            transitionTo(State.OFF)
            return
        }

        if (gp2y && state != State.SHOOTING){
            transitionTo(State.SHOOTING)
            return
        }

        when (state) {
            State.OFF -> {
                if (gp1rb){
                    transitionTo(State.STARTING)
                }
            }
            State.STARTING -> {
                if (getElapsedSec(stateStartTimeNs) >= 0.25){
                    transitionTo(State.INTAKING)
                }
            }
            State.INTAKING -> {
                when(intakeSubState) {
                    INTAKINGSubState.WAIT_FOR_DISTANCE_SENSORS -> {

                        if (topLeftDistance < 95.0 || topRightDistance < 95.0) {
                            intakeSubState = INTAKINGSubState.WAIT_FOR_RAMP_CONTINUOUS
                            conditionContiniousStartTimeNs = 0L
                        }
                    }
                    INTAKINGSubState.WAIT_FOR_RAMP_CONTINUOUS -> {
                        if (rampsActive) {
                            if (conditionContiniousStartTimeNs == 0L) {
                                conditionContiniousStartTimeNs = System.nanoTime()
                            } else if (getElapsedSec(conditionContiniousStartTimeNs) >= 0.5) {
                                transitionTo(State.FINISHING)
                                return
                            }
                        } else {
                            conditionContiniousStartTimeNs = 0L
                        }
                    }
                }
            }
            State.FINISHING -> {
                when (finishingSubState){
                    FINISHINGSubState.INTAKE_SETTLING -> {
                        if (getElapsedSec(stepStartTimeNs) >= 0.05){
                            hw.shooterBallStop.position = SHOOTER_STOP_UP
                            finishingSubState = FINISHINGSubState.FINISH_MOVES
                            stepStartTimeNs = System.nanoTime()
                        }
                    }
                    FINISHINGSubState.FINISH_MOVES -> {
                        if (getElapsedSec(stepStartTimeNs) >= 0.15){
                            transitionTo(State.OFF)
                        }
                    }
                }
            }
            State.SHOOTING -> {
                when (shootingSubState){
                    SHOOTINGSubState.PRE_FLIPPER -> {
                        val stuck = !hw.frontRamp.state && (hw.colorBottomLeft.getDistance(DistanceUnit.MM) < 110.0
                                || hw.colorBottomRight.getDistance(DistanceUnit.MM) < 110.0)
                        if (getElapsedSec(stateStartTimeNs) > 1.0){
                            hw.flipper.position = FLIPPER_UP
                            stepStartTimeNs = System.nanoTime()
                            shootingSubState = SHOOTINGSubState.FLIPPERING
                            return
                        }
                        if (stuck){
                            if (conditionContiniousStartTimeNs == 0L){
                                conditionContiniousStartTimeNs = System.nanoTime()
                            }else if (getElapsedSec(conditionContiniousStartTimeNs) > 0.15){
                                hw.flipper.position = FLIPPER_UP
                                stepStartTimeNs = System.nanoTime()
                                shootingSubState = SHOOTINGSubState.FLIPPERING
                                return
                            }
                        } else{
                            conditionContiniousStartTimeNs = 0L
                        }
                    }
                    SHOOTINGSubState.FLIPPERING -> {
                        if (getElapsedSec(stepStartTimeNs) > 0.4){
                            stepStartTimeNs = System.nanoTime()
                            hw.flipper.position = FLIPPER_DOWN
                            hw.setIntakePower(OUTTAKE_POWER)
                            shootingSubState = SHOOTINGSubState.POST_FLIPPER
                            return
                    }
                    }
                    SHOOTINGSubState.POST_FLIPPER -> {
                        if (getElapsedSec(stepStartTimeNs) > 0.5) {
                            hw.setIntakePower(0.0)
                            hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
                            transitionTo(State.OFF)
                        }
                    }
                }
            }


        }
    }

    private fun onStateEnter(newState: State) {
        clearTimers()
        when (newState) {
            State.OFF -> {
                hw.setIntakePower(0.0)
                hw.bottomBallStop.position = BOTTOM_STOP_STOWED
                hw.flipper.position = FLIPPER_DOWN
                hw.shooterBallStop.position = SHOOTER_STOP_DOWN
            }
            State.STARTING -> {
                hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2)
            }
            State.INTAKING -> {
                hw.setIntakePower(INTAKE_POWER)
                intakeSubState = INTAKINGSubState.WAIT_FOR_DISTANCE_SENSORS
            }
            State.FINISHING -> {
                hw.setIntakePower(0.0)
                hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3)
                finishingSubState = FINISHINGSubState.INTAKE_SETTLING
            }
            State.SHOOTING -> {
                hw.setIntakePower(INTAKE_POWER)
                hw.bottomBallStop.position = BOTTOM_STOP_STOWED
                hw.shooterBallStop.position = SHOOTER_STOP_UP
                hw.prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4)
                shootingSubState = SHOOTINGSubState.PRE_FLIPPER
            }
        }
    }


    private fun getElapsedSec(startTimeNs: Long): Double {
        return (System.nanoTime() - startTimeNs) / 1_000_000_000.0
    }

    private fun transitionTo(newState: State) {
        prevState = state
    }

    private fun clearTimers(){
        val now = System.nanoTime()
        conditionContiniousStartTimeNs = 0L
        stepStartTimeNs = now
        stateStartTimeNs = now
    }
}