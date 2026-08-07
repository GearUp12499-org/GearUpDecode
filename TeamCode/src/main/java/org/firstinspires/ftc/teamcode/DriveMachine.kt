package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class DriveMachine (private val hw: CompBot2Hardware) {
    /*
    all four drive motors belong to this class

    needs:
    joystick input for x, y, rx
    robotAngleRAD (kalman which is technically pinpoint)
     */

    enum class State {
        TICKING,
        AUTO,
        OFF
    }

    private var skew = 0.0

    public lateinit var state: State
    var prevState: State? = null

    var justTransitioned = false

    val removerImpl = REmoverImpl()

    fun init(robotState: RobotState) {
        this.state = State.TICKING
        skew = (if (robotState.red) 1 else -1) * PI / 2


    }

    fun update(robotState: RobotState, input: GamepadState) {

        justTransitioned = state != prevState
        prevState = state

        when (state) {
            State.TICKING -> {

                val angle = -(robotState.ppThetaRad + skew)

                var rotX = input.jx1 * cos(angle) - input.jy1 * sin(angle)
                val rotY = input.jx1 * sin(angle) + input.jy1 * cos(angle)

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

            State.AUTO -> {
                if (justTransitioned) {
                    //uh oh???
                }

                removerImpl.tick(
                    robotState.ppX,
                    robotState.ppY,
                    robotState.ppThetaRad,
                    robotState.velXInch,
                    robotState.velYInch,
                    robotState.angVelRad,
                    robotState.velInch,
                    hw
                )

            }

            State.OFF -> {
                if (hw.frontRight.power != 0.0) {
                    hw.frontRight.power = 0.0
                }
                if (hw.frontLeft.power != 0.0) {
                    hw.frontLeft.power = 0.0
                }
                if (hw.backRight.power != 0.0) {
                    hw.backRight.power = 0.0
                }
                if (hw.backLeft.power != 0.0){
                    hw.backLeft.power = 0.0
                }
            }
        }
    }
}