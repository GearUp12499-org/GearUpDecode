package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.BOTTOM_STOP_STOWED
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.FLIPPER_DOWN
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SHOOTER_STOP_DOWN

class IntakeOFF(private val hw: CompBot2Hardware) : State(){

    override fun onStart() {
        hw.setIntakePower(0.0)
        hw.bottomBallStop.position = BOTTOM_STOP_STOWED
        hw.flipper.position = FLIPPER_DOWN
        hw.shooterBallStop.position = SHOOTER_STOP_DOWN
    }

    override fun onTick() {
        return
    }

    override fun onFinish() {
        return
    }

}