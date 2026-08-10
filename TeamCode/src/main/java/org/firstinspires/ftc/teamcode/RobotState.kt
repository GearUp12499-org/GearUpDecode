package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware.SLIDER_IN
import org.firstinspires.ftc.teamcode.systems.Kalman
import org.firstinspires.ftc.teamcode.systems.Kalman.Companion.INCHES_PER_METER
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.TurretTrack.Companion.TAG_BLUE
import org.firstinspires.ftc.teamcode.systems.TurretTrack.Companion.TAG_RED
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.times

class RobotState(private val hw: CompBot2Hardware, val red: Boolean) {


    //pinpoint items

    val initPose = Pose2D (DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0)
    var ppX: Double = 0.0
    var ppY: Double = 0.0
    var ppThetaDeg: Double = 0.0
    var ppThetaRad: Double = 0.0

    var velXM: Double = 0.0
    var velYM: Double = 0.0

    var velXInch: Double = 0.0
    var velYInch: Double = 0.0
    var velInch: Double = 0.0

    var angVelRad: Double = 0.0

    //turret related
    var turretDeg: Double = 0.0
    var turretRad: Double = 0.0


    //limelight related
    val pipe = if (red) 2 else 7
    val targetTag = if (red) TAG_RED else TAG_BLUE
    var goodLLRead: Boolean = true//used to skip unnesessary steps and decide whether or

    //not to kalman update
    var llFieldX: Double? = 0.0
    var llFieldY: Double? = 0.0
    var llFieldTheta: Double? = 0.0

    var llDistance: Double? = 0.0 //if null for 2 sec ll only resets pose to 0
    var tx: Double? = 0.0

    //kalman related

    lateinit var kalman: KalmanImpl

    var kalmanX: Double = 0.0
    var kalmanY: Double = 0.0
    var kalmanTheta: Double = 0.0

    var kalmanPose: REmover.RobotPose = REmover.RobotPose(0.0, 0.0, 0.0)

    //sensor related
    var colorTopRight: Double = 0.0
    var colorTopLeft: Double = 0.0

    var colorBottomLeft: Double = 0.0
    var colorBottomRight: Double = 0.0

//    private val I2C_ADDR = I2cAddr.create8bit(0x52)
//    private lateinit var i2cTopRight: I2cDeviceSynch
//    private lateinit var i2cTopLeft: I2cDeviceSynch
//    private lateinit var i2cBottomRight: I2cDeviceSynch
//    private lateinit var i2cBottomLeft: I2cDeviceSynch

    var frontRamp: Boolean = false
    var middleRamp: Boolean = false

    var shoot1Vel: Double = 0.0


    fun init() {
        hw.limelight.start()
        hw.limelight.pipelineSwitch(pipe)

        hw.pinpoint.resetPosAndIMU()
        hw.pinpoint.setPosition(Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0))
        kalman = KalmanImpl(
            initPose.getX(DistanceUnit.INCH),
            initPose.getY(DistanceUnit.INCH),
            initPose.getHeading(AngleUnit.RADIANS),
            red
        )
        kalman.init()

        hw.slider.position = SLIDER_IN

//        i2cTopRight = hw.colorTopRight as I2cDeviceSynch
//        i2cTopLeft = hw.colorTopLeft as I2cDeviceSynch
//        i2cBottomRight = hw.colorBottomRight as I2cDeviceSynch
//        i2cBottomLeft = hw.colorBottomLeft as I2cDeviceSynch
//
//        i2cTopLeft.readWindow
//
//        val sensors = listOf(i2cTopRight, i2cTopLeft, i2cBottomRight, i2cBottomLeft)
//        for (sensor in sensors){
//            sensor.i2cAddress = I2C_ADDR
//            sensor.logging = false
//
//            val readWindow = I2cDeviceSynch.ReadWindow(
//                0x1C,1, I2cDeviceSynch.ReadMode.REPEAT
//            )
//
//            sensor.readWindow = readWindow
//            sensor.engage()
//
//        }
    }


    fun updateState() {

        //update pinpoint
        hw.pinpoint.update()

        val ppPose = hw.pinpoint.position
        ppX = ppPose.getX(DistanceUnit.INCH)
        ppY = ppPose.getY(DistanceUnit.INCH)
        ppThetaRad = ppPose.getHeading(AngleUnit.RADIANS)
        ppThetaDeg = ppPose.getHeading(AngleUnit.DEGREES)

        velXM = hw.pinpoint.getVelX(DistanceUnit.METER)
        velYM = hw.pinpoint.getVelY(DistanceUnit.METER)

        velXInch = hw.pinpoint.getVelX(DistanceUnit.INCH)
        velYInch = hw.pinpoint.getVelY(DistanceUnit.INCH)
        velInch = hypot(velXInch, velYInch)

        angVelRad = hw.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)

        //update turret
        turretDeg =
            (-hw.turretEncoder.getCurrentPosition() / CompBot2Hardware.TICKS_PER_DEG)
        turretRad = turretDeg * (PI / 180)

        //update limelight
        goodLLRead = true

        val result = hw.limelight.latestResult

        if (result == null || !result.isValid) {
            goodLLRead = false
        } else if (result.pipelineIndex != pipe) {
            hw.limelight.pipelineSwitch(pipe)
            goodLLRead = false
        }

        if (goodLLRead) {
            hw.limelight.updateRobotOrientation(ppThetaDeg + turretDeg)
            val botpose = result.botpose_MT2

            val limelightX: Double = botpose.getPosition().x * INCHES_PER_METER * -1
            val limelightY: Double = botpose.getPosition().y * INCHES_PER_METER * -1

            val llFieldPose = CompBot2Hardware.getPoseRobotFromLL(
                limelightX,
                limelightY,
                turretRad,
                ppThetaRad
            )
            llFieldX = llFieldPose.first
            llFieldY = llFieldPose.second
            llFieldTheta = llFieldPose.third

            val tags = result.fiducialResults
            val target = tags.firstOrNull { it.fiducialId == targetTag }
            if (target != null) {
                llDistance = CompBot2Hardware.taToDistance(target.targetArea)
                tx = -target.targetXDegrees
            } else {
                llDistance = null
                tx = null
            }
        } else {
            llFieldX = null
            llFieldY = null
            llFieldTheta = null
            llDistance = null
            tx = null
        }

        //update kalman position

        kalman.tickKalman(
            turretDeg,
            ppX,
            ppY,
            ppThetaRad,
            velInch,
            goodLLRead,
            llFieldX,
            llFieldY,
            llFieldTheta
        )
        kalmanX = kalman.stateX
        kalmanY = kalman.stateY
        kalmanTheta = kalman.stateTheta
        kalmanPose = kalman.kalmanPose2D

        //update sensors

        //5ms/read
        //round robin read? read 1 ever loop and cycle the one that is read
        colorTopRight = hw.colorTopRight.getDistance(DistanceUnit.MM)
        colorTopLeft = hw.colorTopLeft.getDistance(DistanceUnit.MM)
        colorBottomLeft = hw.colorBottomLeft.getDistance(DistanceUnit.MM)
        colorBottomRight = hw.colorBottomRight.getDistance(DistanceUnit.MM)


        frontRamp = hw.frontRamp.state
        middleRamp = hw.middleRamp.state

        shoot1Vel = hw.shoot1Vel


    }


}