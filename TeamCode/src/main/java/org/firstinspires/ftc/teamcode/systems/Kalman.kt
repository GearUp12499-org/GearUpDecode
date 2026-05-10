package org.firstinspires.ftc.teamcode.systems

import android.util.Log
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.systemPackages
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.PoseSet
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.TurretTrack.Companion.TAG_BLUE
import org.firstinspires.ftc.teamcode.systems.TurretTrack.Companion.TAG_RED
import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class Kalman(
    private val hw: CompBot2Hardware,
    private val red: Boolean,
    var initX: Double,
    var initY: Double,
    var initTheta: Double
) : Task<Kalman>() {

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

    val structuralErrorX = 0.0
    val structuralErrorY = 0.0


    var llx = 0.0
    var lly = 0.0

    var inMotion = false //if true, don't update with limelight
    var hasRead = false //if true, don't read in this movement

    var updateCounter = 0


    init{
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
    }
    companion object {
        init {
            systemPackages.add(Kalman::class.qualifiedName!!)
        }

        const val INCHES_PER_METER = 39.37

        const val TICKS_PER_DEG = CompBot2Hardware.TICKS_PER_DEG
    }

    override fun onStart() {
        hw.limelight.start()
        hw.limelight.pipelineSwitch(pipe)

    }

    override fun onTick(): Boolean {
        Log.i("Kalman","Running")

        hw.pinpoint.update()

        val velocity = hypot(hw.pinpoint.getVelX(DistanceUnit.INCH),hw.pinpoint.getVelX(DistanceUnit.INCH))

        if (velocity > 1){
            inMotion = true
            hasRead = false
        } else {1
            inMotion = false
        }

        val pinpointPose = hw.pinpoint.position

        val currentX = pinpointPose.getX(DistanceUnit.INCH)
        val currentY = pinpointPose.getY(DistanceUnit.INCH)
        val currentTheta = pinpointPose.getHeading(AngleUnit.RADIANS)

        val dx = currentX - prevX
        val dy = currentY - prevY
        val dTheta = currentTheta - prevTheta

        predict(dx, dy, dTheta, currentTheta)

        prevX = currentX
        prevY = currentY
        prevTheta = currentTheta

        if (hasRead || inMotion){
            return false
        }

        val result = hw.limelight.latestResult


        //safety to make sure you are in the right pipeline
        val actualPipeline = result.pipelineIndex
        if (actualPipeline != pipe) {
            Log.w(
                this::class.simpleName,
                "Wrong pipeline ($actualPipeline), trying to switch to $pipe"
            )
            hw.limelight.pipelineSwitch(pipe)
            Log.i("Kalman","wrong pipeline")
            return false
        }


        if (result == null) {
            Log.i("Kalman", "limelight null")
            return false
        }

        if (!result.isValid) {
            Log.i("Kalman", result.ta.toString())
            Log.i("Kalman","limelight not valid")
            return false
        }



        hw.limelight.updateRobotOrientation((pinpointPose.getHeading(AngleUnit.DEGREES) + (-hw.turretEncoder.getCurrentPosition() / TICKS_PER_DEG)))
        val botpose = result.botpose_MT2


        val limelightX: Double = botpose.getPosition().x * INCHES_PER_METER * -1
        val limelightY: Double = botpose.getPosition().y * INCHES_PER_METER * -1
        val thetaTurretRelTurret = (-hw.turretEncoder.getCurrentPosition() / TICKS_PER_DEG) * (PI / 180)

        var (llFieldX, llFieldY, llFieldTheta) = getPoseRobotFromLL(limelightX, limelightY, thetaTurretRelTurret, kalmanState.get(2,0))

        llFieldX -= structuralErrorX
        llFieldY -= structuralErrorY

        llx = llFieldX
        lly = llFieldY

        //make R
        var R = SimpleMatrix(
            arrayOf<DoubleArray?>(
                doubleArrayOf(0.0281, 0.0320, 0.0),
                doubleArrayOf(0.0320, 0.1529, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0)
            )
        )

        update(llFieldX, llFieldY, llFieldTheta, R)
        updateCounter += 1
        hasRead = true

        return false
    }

    //limelight stuff
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE
    val targetTag = if (red) TAG_RED else TAG_BLUE
    val targetPose = poseSet.goalAT
    val pipe = if (red) 2 else 7
    val flipIfBlue = if (red) 1.0 else -1.0

    private fun getPoseRobotFromLL(
        xLL: Double,
        yLL: Double,
        thetaTurret: Double,
        thetaRobot: Double
    ): Triple<Double, Double, Double> {
        // thetaRobot MUST be in radians

        val rTurret = 6.5
        val tOffset = 0.5

        val d = sqrt(
            (rTurret).pow(2.0) +
                    (tOffset).pow(2.0) -
                    2 * (rTurret) * (tOffset) * cos(PI - thetaTurret)
        )

        val x = asin(
            sin(PI - thetaTurret) * rTurret / d
        )

        val f = d * cos(x)
        val s = d * sin(x)

        val xOff = f * cos(thetaRobot) - s * sin(thetaRobot)
        val yOff = f * sin(thetaRobot) + s * cos(thetaRobot)

        val xRobot = xLL + xOff
        val yRobot = yLL + yOff


        //use fiducial tag to get relative to goal angle
        val tags = hw.limelight.latestResult.fiducialResults
        val target = tags.firstOrNull { it.fiducialId == targetTag }
        //just use pinpoint angle if no fiducial tag is found
        if (target == null) {
            return Triple(xRobot, yRobot, thetaRobot)
        }

        //THIS IS VERY LIKELY WRONG :)
        val turretAngle = PI - ((-hw.turretEncoder.getCurrentPosition() / TICKS_PER_DEG) * (PI/180))
        val atan2Angle = atan2(poseSet.goalAT.y - yLL, poseSet.goalAT.x - xLL)
        val bearing = (target.targetXDegrees)*(PI/180)
        Log.i("KalmanMath","start")
        Log.i("KalmanMath",turretAngle.toString())
        Log.i("KalmanMath",atan2Angle.toString())
        Log.i("KalmanMath",bearing.toString())
        var llAngle = turretAngle + atan2Angle + bearing
        llAngle %= (2*PI)
        if (llAngle > PI) {
            llAngle -= 2*PI
        }
        if (llAngle < -PI) {
            llAngle += 2*PI
        }
        Log.i("KalmanMath", llAngle.toString())
        return Triple(xRobot, yRobot, thetaRobot)
    }


//getters
    val stateX: Double
        get() = kalmanState.get(0, 0)

    val stateY: Double
        get() = kalmanState.get(1, 0)

    val stateTheta: Double
        get() = kalmanState.get(2, 0)

    val limelightx: Double
        get() = llx

    val limelighty: Double
        get() = lly


    fun predict(dx: Double, dy: Double, dTheta: Double, pinpointTheta: Double) {
        Log.i("Kalman","predicting")

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
        Log.i("Kalman","updating")

        val measurement = SimpleMatrix(doubleArrayOf(zx, zy, zTheta))

        //R is measurement error, we can customize with data to change how much the measurement is trusted

        val kalmanGain = P.mult((P.plus(R)).invert())

        kalmanState = kalmanState.plus(kalmanGain.mult((measurement.minus(kalmanState))))

        val identity = SimpleMatrix.identity(3)
        P = (identity.minus(kalmanGain)).mult(P)
    }


}