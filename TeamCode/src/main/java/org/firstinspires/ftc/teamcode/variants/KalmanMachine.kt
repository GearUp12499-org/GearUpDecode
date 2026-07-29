package org.firstinspires.ftc.teamcode.variants

import com.qualcomm.hardware.limelightvision.LLResult
import org.firstinspires.ftc.teamcode.KalmanImpl

class KalmanMachine{

    /*
    things that belong to this class

    sensor inputs that this class needs
     */

    public enum class State {
        TICKING
    }

    var state = State.TICKING
        private set

    private lateinit var kalman: KalmanImpl

    fun init(
        initX: Double,
        initY: Double,
        initTheta: Double,
        red: Boolean
    ){
        kalman = KalmanImpl(initX, initY, initTheta, red)
    }

    fun update(
        currentTurret: Double,
        ppX: Double,
        ppY: Double,
        ppTheta: Double,
        velocity: Double,
        result: LLResult?,
        llFieldX: Double,
        llFieldY: Double,
        llFieldTheta: Double
    ){
        kalman.tickKalman(
            currentTurret,
            ppX,
            ppY,
            ppTheta,
            velocity,
            result,
            llFieldX,
            llFieldY,
            llFieldTheta
        )
            }
        }



