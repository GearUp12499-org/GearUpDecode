package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

interface Encoder {
    var isFlipped: Boolean
    val flippedFactor: Int
        get() = if (isFlipped) -1 else 1

    fun getCurrentPosition(): Int
    fun getVelocity(): Double
    fun reset()
}

class MotorEncoder(val motor: DcMotorEx): Encoder {
    private var isFlippedBacking = false
    override var isFlipped: Boolean
        get() = isFlippedBacking
        set(value) { isFlippedBacking = value }

    override fun getCurrentPosition(): Int = motor.currentPosition * flippedFactor
    override fun reset() {
        val modeBefore = motor.mode
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = modeBefore
    }

    override fun getVelocity(): Double {
        return motor.velocity * flippedFactor
    }
}