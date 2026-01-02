package org.firstinspires.ftc.teamcode.systems

import io.github.gearup12499.taskshark.Task
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware

class ShooterImpl(private val hw: CompBot2Hardware): Task<ShooterImpl>() {
    override fun onStart() {
        hw.setupShooterVel()
    }

    override fun onTick(): Boolean {
        hw.copyShooterPower()
        return false
    }

    fun setTarget(vel: Double) {
        hw.setShoot1Vel(vel)
    }
}