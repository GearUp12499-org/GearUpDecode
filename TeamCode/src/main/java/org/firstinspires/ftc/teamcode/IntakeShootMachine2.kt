package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_UP
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_UP

class IntakeShootMachine2 (private val hw: CompBot2Hardware) {

    enum class IntakeState{
        IDLE,
        INTAKING,
        REVERSING
    }
    lateinit var intakeState: IntakeState
    var prevIntakeState = IntakeState.IDLE

    enum class FlipperState{
        DOWN,
        UP
    }
    lateinit var flipperState: FlipperState
    var prevFlipperState = FlipperState.UP

    enum class ShooterStopState{
        DOWN,
        UP
    }
    lateinit var shooterStopState: ShooterStopState
    var prevShooterStopState = ShooterStopState.UP

    companion object {
        val INTAKE_POWER = 1.0
        val OUTTAKE_POWER = -0.8
    }

    fun init(){
        intakeState = IntakeState.INTAKING
        flipperState = FlipperState.DOWN
        shooterStopState = ShooterStopState.DOWN
    }

    fun update(){

        if (intakeState != prevIntakeState){
            intakeOnEnter(intakeState)
        }
        prevIntakeState = intakeState

        if (flipperState != prevFlipperState){
            flipperOnEnter(flipperState)
        }
        prevFlipperState = flipperState

        if (shooterStopState != prevShooterStopState){
            shooterStopOnEnter(shooterStopState)
        }
        prevShooterStopState = shooterStopState
    }

    fun intakeOnEnter(newState: IntakeState) {
        when (newState){
            IntakeState.IDLE -> {
                hw.setIntakePower(0.0)
            }
            IntakeState.INTAKING -> {
                hw.setIntakePower(INTAKE_POWER)
            }
            IntakeState.REVERSING -> {
                hw.setIntakePower(OUTTAKE_POWER)
            }
        }
    }

    fun flipperOnEnter(newState: FlipperState){
        when (newState){
            FlipperState.DOWN -> hw.flipper.position = FLIPPER_DOWN
            FlipperState.UP -> hw.flipper.position = FLIPPER_UP
        }
    }
    fun shooterStopOnEnter(newState: ShooterStopState){
        when (newState){
            ShooterStopState.DOWN -> hw.shooterBallStop.position = SHOOTER_STOP_DOWN
            ShooterStopState.UP -> hw.shooterBallStop.position = SHOOTER_STOP_UP
        }
    }

    fun intakeTransitionTo(state: IntakeState){
        intakeState = state
    }

    fun flipperTransitionTo(state: FlipperState){
        when (state){
            FlipperState.UP -> {
                if (shooterStopState == ShooterStopState.DOWN){
                    telemetry.addData("CAN'T FLIPPER WHEN THE SHOOTERSTOP IS DOWN","")
                    return
                }
            }
            else -> {}
        }
        flipperState = state
    }

    fun shooterStopTransitionTo(state: ShooterStopState){
        shooterStopState = state
    }

}