package org.firstinspires.ftc.teamcode

import android.util.Log
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_UP

class IntakeMachine(private val hw: CompBot2Hardware) {
    enum class State{
        OFF,
        INTAKING_TO_DISTANCE_SENSORS,
        INTAKING_TO_RAMPS,
        FINISHING,
        OUTTAKE
    }
    var state = State.OFF
    var prevState = state
    private var CCSTNS: Long = 0L
    private var stateStartTimeNS: Long = 0L
    var justTransitioned = true
    fun update(a:Boolean,b:Boolean){
        Log.i("State",state.toString())
        justTransitioned = (state != prevState)
        prevState = state
        if (justTransitioned){
            clearTimers()
        }
        when (state) {
            State.OFF -> {
                if (justTransitioned){
                    hw.setIntakePower(0.0)
                    hw.shooterBallStop.position = SHOOTER_STOP_UP
                }
                if(a){
                    state = State.INTAKING_TO_RAMPS
                }
                if(b){
                    state = State.OUTTAKE
                }
            }
            State.INTAKING_TO_DISTANCE_SENSORS -> {
                if (justTransitioned){
                    hw.setIntakePower(1.0)
                    hw.shooterBallStop.position = SHOOTER_STOP_DOWN
                    hw.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2)

                }
                if (hw.colorTopLeft.getDistance(DistanceUnit.MM) < 95.0 || hw.colorTopRight.getDistance(DistanceUnit.MM) < 95.0){
                    state = State.INTAKING_TO_RAMPS
                    CCSTNS = 0L
                }

            }
            State.INTAKING_TO_RAMPS -> {
                if(justTransitioned){
                    hw.setIntakePower(1.0)
                    hw.shooterBallStop.position = SHOOTER_STOP_DOWN
                    hw.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2)
                }
                if(hw.frontRamp.state){
                    if(CCSTNS == 0L){
                        CCSTNS = System.nanoTime()
                    }else if(getElapsedSeconds(CCSTNS) >= 0.5){
                        state = State.FINISHING
                    }
                }else{
                    CCSTNS = 0L
                }
                Log.i("Ramps",(hw.frontRamp.state).toString())
                Log.i("CCSTNS",CCSTNS.toString())
                Log.i("Elapsed",getElapsedSeconds(CCSTNS).toString())
                Log.i("Front",(hw.frontRamp.state).toString())
            }
            State.FINISHING -> {
                if(justTransitioned){
                    hw.setIntakePower(0.0)
                    hw.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3)
                }
                if(getElapsedSeconds(stateStartTimeNS) >= 0.15){
                    state = State.OFF
                }

            }
            State.OUTTAKE -> {
                if(justTransitioned){
                    hw.setIntakePower(-1.0)
                    hw.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7)

                }
                if(!hw.frontRamp.state && !hw.middleRamp.state){
                    if(CCSTNS == 0L){
                        CCSTNS = System.nanoTime()
                    }else if(getElapsedSeconds(CCSTNS) >= 0.5){
                        state = State.FINISHING
                    }
                }else{
                    CCSTNS = 0L
                }
                Log.i("Front",(hw.frontRamp.state).toString())
                Log.i("Middle",(hw.middleRamp.state).toString())
            }
        }
    }
    fun getElapsedSeconds(startTime_ns:Long): Double{
        return (System.nanoTime() - startTime_ns)/1000000000.0
    }
    fun clearTimers(){
        CCSTNS = 0L
        stateStartTimeNS = System.nanoTime()
    }
}