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
    fun update(robotState: RobotState, input: GamepadState){

        when(state){
            State.TICKING -> {
                var rotX = input.jx1 * cos(-robotState.ppThetaRad) - input.jy1 * sin(-robotState.ppThetaRad)
                val rotY = input.jx1 * sin(-robotState.ppThetaRad) + input.jy1 * cos(-robotState.ppThetaRad)

                rotX *= 1.1 // Counteract imperfect strafing

                val denominator = max(abs(rotY) + abs(rotX) + abs(input.rx1), 1.0)
                val frontLeftPower = (rotY + rotX + input.rx1) / denominator
                val backLeftPower = (rotY - rotX + input.rx1) / denominator
                val frontRightPower = (rotY - rotX - input.rx1) / denominator
                val backRightPower = (rotY + rotX - input.rx1) / denominator

                hw.frontLeft.power = frontLeftPower
                hw.backLeft.power = backLeftPower
                hw.frontRight.power = frontRightPower
                hw.backRight.power = backRightPower
            }
        }
    }
}