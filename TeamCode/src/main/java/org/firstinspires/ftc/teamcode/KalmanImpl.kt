package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.hardware.limelightvision.LLResult
import org.ejml.data.SingularMatrixException
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.REmover
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

class KalmanImpl (
    val initX: Double,
    val initY: Double,
    val initTheta: Double,
    val red: Boolean
){
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE

    companion object {
        const val INCHES_PER_METER = 39.27
        const val TICKS_PER_DEG = CompBot2Hardware.TICKS_PER_DEG
    }
    //3x1 matrix {x, y, theta}
    var kalmanState = SimpleMatrix(3, 1)

    //3x3 matrix, describes uncertainty in your pose estimation/prediction
    private var P = SimpleMatrix(3, 3)

    //3x3 matrix, describes how confident you are in your prediction model, lower numbers = more confident
    //get better values later
    //Q is how much you are adding to the covariance every loop, pinpoint drift + bump?
    private val Q = SimpleMatrix(
        arrayOf<DoubleArray?>(
            doubleArrayOf(0.001, 0.0, 0.0),
            doubleArrayOf(0.0, 0.001, 0.0),
            doubleArrayOf(0.0, 0.0, 0.000011)
        )
    )

    // x, y, theta or the last pinpoint read

    var prevX = 0.0
    var prevY = 0.0
    var prevTheta = 0.0

    var structuralErrorX = 0.0
    var structuralErrorY = 0.0


    var llx = 0.0
    var lly = 0.0

    var inMotion = false //if true, don't update with limelight
    var hasRead = false //if true, don't read in this movement

    var updateCounter = 0

    var prevTurret = 0.0

    private var lastTime = 0L

    var startTime: Long = 0

    var prevLLX = 0.0

    var prevLLY = 0.0

    var counter = 0

    var reverse = 1.0


    fun init() {
        //give initial position
        kalmanState.set(0, initX)
        kalmanState.set(1, initY)
        kalmanState.set(2, initTheta)
        //initialize P values as a diagonal, P doesn't necessarily stay a diagonal but assume they begin as uncorrelated
        for (i in 0..2) {
            P.set(i, i, 0.1)
        }
        prevX = initX
        prevY = initY
        prevTheta = initTheta

        if (!red){
            reverse = -1.0
        }
    }

    fun tickKalman(
        currentTurret: Double,
        currentX: Double,
        currentY: Double,
        currentTheta: Double,
        velocity: Double,
        result: LLResult?,
        llFieldX: Double,
        llFieldY: Double,
        llFieldTheta: Double
        ) {

        var llFieldX = llFieldX
        var llFieldY = llFieldY
        var llFieldTheta = llFieldTheta

        val now = System.nanoTime()
        var dt = 0.0
        if (lastTime != 0L) {
            dt = (now - lastTime) / 1e9
        }
        lastTime = now

        val deltaTurret = (currentTurret - prevTurret) / dt
        prevTurret = currentTurret

        val currentThetaDeg = currentTheta * 180 / PI
        val dx = currentX - prevX
        val dy = currentY - prevY
        val dTheta = currentTheta - prevTheta

        predict(dx, dy, dTheta, currentTheta)

        prevX = currentX
        prevY = currentY
        prevTheta = currentTheta

        if (velocity > 1 || abs(deltaTurret) > 100.0) {
            startTime = (now / 1e9).toLong()
            hasRead = false
            counter = 0
            return
        }
        if (result == null) {
            Log.i("Kalman", "limelight null")
            return
        }

        if (!result.isValid) {
            Log.i("Kalman", result.ta.toString())
            Log.i("Kalman", "limelight not valid")
            return
        }
        //if the reading is very different from the last one
        if ((abs(prevLLX - llFieldX) > 1.0) || (abs(prevLLY - llFieldY) > 1.0)) {
            counter = 0
//            Log.i("llx",llFieldX.toString())
//            Log.i("lly",llFieldY.toString())
//            Log.i("prevLLX",prevLLX.toString())
//            Log.i("prevLLY",prevLLY.toString())
//            Log.i("diffLLX", (prevLLX-llx).toString())
//            Log.i("diffLLY",(prevLLY-lly).toString())
        } else {
            counter += 1
//            Log.i("madeIt","")
        }
//        Log.i("counter",counter.toString())

        prevLLX = llFieldX
        prevLLY = llFieldY

        if (counter <= 4 || hasRead) {
            return
        }
        //make R

        //middle area
        var R = SimpleMatrix(
            arrayOf<DoubleArray?>(
                doubleArrayOf(2.0, -0.5 * reverse, 0.0),
                doubleArrayOf(-0.5 * reverse, 2.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.00001)
            )
        )
        structuralErrorX = -1.069
        structuralErrorY = 1.439 * reverse

        //far wall

        if (stateX > 48) {
            R = SimpleMatrix(
                arrayOf<DoubleArray?>(
                    doubleArrayOf(1.0, 0.0320, 0.0),
                    doubleArrayOf(0.0320, 4.0, 0.0),
                    doubleArrayOf(0.0, 0.0, 0.00001)
                )
            )

            structuralErrorX = 0.1606325833
            structuralErrorY = 1.280544028 * reverse
        }

        //red side wall
        else if ((stateY < -48) && red) {
            R = SimpleMatrix(
                arrayOf<DoubleArray?>(
                    doubleArrayOf(4.0, 0.0320, 0.0),
                    doubleArrayOf(0.0320, 1.0, 0.0),
                    doubleArrayOf(0.0, 0.0, 0.00001)
                )
            )

            structuralErrorX = -0.682
            structuralErrorY = 2.203 * reverse
        }
        //blue side wall
        else if ((stateY > 48) && !red){
            R = SimpleMatrix(
                arrayOf<DoubleArray?>(
                    doubleArrayOf(4.0, 0.0320, 0.0),
                    doubleArrayOf(0.0320, 1.0, 0.0),
                    doubleArrayOf(0.0, 0.0, 0.00001)
                )
            )
        }

        llFieldX -= structuralErrorX
        llFieldY -= structuralErrorY

        if (stateX <= -24.0) {
            return
        }

        if ((abs(llFieldX) > 72.0) || abs(llFieldY) > 72.0) {
            return
        }
        update(llFieldX, llFieldY, llFieldTheta, R)
        updateCounter += 1
        hasRead = true
        counter = 0
        return
    }

    fun predict(dx: Double, dy: Double, dTheta: Double, pinpointTheta: Double) {
//        Log.i("Kalman","predicting")

        //rotation matrix from pinpoint field to state field

        val rotationTheta = pinpointTheta - kalmanState.get(2, 0)

        val rotatedX = dx * cos(rotationTheta) - dy * sin(rotationTheta)
        val rotatedY = dx * sin(rotationTheta) + dy * cos(rotationTheta)

        val u = SimpleMatrix(doubleArrayOf(dx, dy, dTheta))
        //predict the next state based on how much pinpoint says you have moved since last call
        //this is better than using field position because once the filter corrects the poses
        //after a bump all the pinpoint readings will still be accurate
        kalmanState = kalmanState.plus(u)
        //predict the P covariance: P(k,k+1)=AP(k,K)A(transpose)+Q, in this case A is an identity matrix so P(k,k+1)=P(k,k)+Q
        P = P.plus(Q)
    }


    fun update(zx: Double, zy: Double, zTheta: Double, R: SimpleMatrix) {
//        Log.i("Kalman","updating")

        val prevStateX = stateX
        val prevStateY = stateY

        val measurement = SimpleMatrix(doubleArrayOf(zx, zy, zTheta))

        //R is measurement error, we can customize with data to change how much the measurement is trusted

        val kalmanGain = try {
            P.mult((P.plus(R)).invert())
        } catch (e: SingularMatrixException) {
            Log.e("Kalman", "singular matrix exception", e)
            return
        }

        kalmanState = kalmanState.plus(kalmanGain.mult((measurement.minus(kalmanState))))

        if ((abs(stateX - prevStateX) > 15.0) || (abs(stateY - prevStateY) > 15.0)){
            kalmanState.set(0, prevStateX)
            kalmanState.set(1, prevStateY)
            Log.i("kalmanError", stateX.toString())
            Log.i("kalmanError", stateY.toString())
        }

        //   Log.i("kalmanGain",kalmanGain.toString())

        val identity = SimpleMatrix.identity(3)
        P = (identity.minus(kalmanGain)).mult(P)
    }


    val stateX: Double
        get() = kalmanState.get(0, 0)

    val stateY: Double
        get() = kalmanState.get(1, 0)

    val stateTheta: Double
        get() = kalmanState.get(2, 0)

    val kalmanPose2D: REmover.RobotPose
        get() = REmover.RobotPose(stateX, stateY, stateTheta)

    val distance: Double
        get() = (hypot((stateX - poseSet.goalAT.x), (stateY - poseSet.goalAT.y)) - 1.0)



}