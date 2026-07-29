package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class DriveMachine (private val hw: CompBot2Hardware){
    /*
    all four drive motors belong to this class

    needs:
    joystick input for x, y, rx
     */

    public enum class State{
        TICKING
    }

    public lateinit var state: State

    fun init(){
        this.state = State.TICKING
    }
    /*
    angle in rad
     */
    fun update(x: Double, y: Double, rx: Double, angle: Double){

        if (this.state == State.TICKING){
            var rotX = x * cos(-angle) - y * sin(-angle) //why is angle negative here?
            val rotY = x * sin(-angle) + y * cos(-angle)

            rotX *= 1.1 // Counteract imperfect strafing

            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
            val frontLeftPower = (rotY + rotX + rx) / denominator
            val backLeftPower = (rotY - rotX + rx) / denominator
            val frontRightPower = (rotY - rotX - rx) / denominator
            val backRightPower = (rotY + rotX - rx) / denominator

            hw.frontLeft.power = frontLeftPower
            hw.backLeft.power = backLeftPower
            hw.frontRight.power = frontRightPower
            hw.backRight.power = backRightPower

        }

    }


}