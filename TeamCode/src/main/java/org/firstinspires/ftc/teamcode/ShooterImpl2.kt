package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware

class ShooterImpl2 (private val hw: CompBot2Hardware) {

    companion object{
        private const val ACCEPTABLE_VELOCITY_DIFF = 30.0
    }

    var target = 0.0

    val defaultPushThreshold = 200
    var pushThreshold = defaultPushThreshold

    var mode = false

    fun init(){
        hw.setupShooterVel()
    }

    fun tickShooter(shoot1Velk: Double){
        val targetMode = target - hw.shoot1Vel <= pushThreshold
        if (targetMode != mode) if (targetMode) hw.shoot1Vel = target
        mode = targetMode
        if (!mode)
            hw.setShooterPower(1.0)
        hw.copyShooterPower()
        return
    }

    fun setTarget(vel: Double){
        target = vel
        mode = false
    }
}