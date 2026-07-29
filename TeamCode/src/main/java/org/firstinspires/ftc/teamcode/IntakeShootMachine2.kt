package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.BOTTOM_STOP_STOWED
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_DOWN

class IntakeShootMachine2 (private val hw: CompBot2Hardware) {

    public enum class State{
        OFF,
        STARTING,
        INTAKING,
        FINISHING,
        SHOOTING
    }

    private val Off = IntakeOFF(hw)

    public lateinit var state: State

    private var prevState = State.OFF

    private val INTAKE_POWER = 1.0

    private val waiter = Wait()

    fun init() {
        state = State.OFF
        hw.setIntakePower(0.0)
        hw.bottomBallStop.position = BOTTOM_STOP_STOWED
        hw.flipper.position = FLIPPER_DOWN
        hw.shooterBallStop.position = SHOOTER_STOP_DOWN

    }

    fun update(gp1rb: Boolean){

        when (state) {
            State.OFF -> {

            }
        }


    }
}