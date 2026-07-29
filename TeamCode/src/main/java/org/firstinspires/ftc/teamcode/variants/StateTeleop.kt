package org.firstinspires.ftc.teamcode.variants

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.AimingMachine
import org.firstinspires.ftc.teamcode.DriveMachine
import org.firstinspires.ftc.teamcode.IntakeShootMachine
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.Kalman.Companion.INCHES_PER_METER
import org.firstinspires.ftc.teamcode.systems.Kalman.Companion.TICKS_PER_DEG
import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

abstract class StateTeleop (private val red: Boolean): LinearOpMode(){

    private lateinit var hw: CompBot2Hardware
    val pipe = if (red) 2 else 7


    override fun runOpMode() {
        hw = CompBot2Hardware(hardwareMap)

        val driveMachine = DriveMachine(hw)
        val intakeShootMachine = IntakeShootMachine(hw)
        val aimingMachine = AimingMachine(hw)

        waitForStart()

        driveMachine.init()
        intakeShootMachine.init()
        aimingMachine.init()

        hw.limelight.start()
        hw.limelight.pipelineSwitch(pipe)

        var loopStartTime = System.nanoTime()
        val looptimeLimitMs = 100.0

        while(opModeIsActive()){

            //DO ALL SENSOR READS HERE
            //update pinpoint
            hw.pinpoint.update()
            val ppPose = hw.pinpoint.position
            val ppX = ppPose.getX(DistanceUnit.INCH)
            val ppY = ppPose.getY(DistanceUnit.INCH)
            val ppThetaRad = ppPose.getHeading(AngleUnit.RADIANS)
            val ppThetaDeg = ppPose.getHeading(AngleUnit.DEGREES)

            val velXM = hw.pinpoint.getVelX(DistanceUnit.METER)
            val velYM = hw.pinpoint.getVelY(DistanceUnit.METER)

            val velXInch = hw.pinpoint.getVelX(DistanceUnit.INCH)
            val velYInch = hw.pinpoint.getVelY(DistanceUnit.INCH)
            val velInch = hypot(velXInch, velYInch)

            //update turret
            //seriously you need this

            //update limelight
            val result = hw.limelight.latestResult

            val actualPipeline = result.pipelineIndex
            if (actualPipeline != pipe) {
                hw.limelight.pipelineSwitch(pipe)
            }
            hw.limelight.updateRobotOrientation(ppThetaDeg +)//FINISH THIS WHEN YOU DO TURRET
            val botpose = result.botpose_MT2

            val limelightX: Double = botpose.getPosition().x * INCHES_PER_METER * -1
            val limelightY: Double = botpose.getPosition().y * INCHES_PER_METER * -1

//            val thetaTurretRelTurret = error//who named this  ;;
            val (llFieldX, llFieldY, llFieldTheta) = getPoseRobotFromLL(
                limelightX,
                limelightY,
                thetaTurretRelTurret,
                ppThetaRad
            )

            //update kalman position
//          //update gamepad

            //update sensors



            driveMachine.update()
            intakeShootMachine.update()
            aimingMachine.update()

            if (System.nanoTime()/1_000_000 - loopStartTime < looptimeLimitMs){
                sleep(1)
            }
            loopStartTime = System.nanoTime()

        }
    }


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
        val tags = ll.latestResult.fiducialResults
        val target = tags.firstOrNull { it.fiducialId == targetTag }
        //just use pinpoint angle if no fiducial tag is found
        if (target == null) {
            return Triple(xRobot, yRobot, thetaRobot)
        }

        //THIS IS VERY LIKELY WRONG :)
        val turretAngle =
            PI - ((-hw.turretEncoder.getCurrentPosition() / TICKS_PER_DEG) * (PI / 180))
        val atan2Angle = atan2(poseSet.goalAT.y - yLL, poseSet.goalAT.x - xLL)
        val bearing = (target.targetXDegrees) * (PI / 180)
//        Log.i("KalmanMath","start")
//        Log.i("KalmanMath",turretAngle.toString())
//        Log.i("KalmanMath",atan2Angle.toString())
//        Log.i("KalmanMath",bearing.toString())
        var llAngle = turretAngle + atan2Angle + bearing
        llAngle %= (2 * PI)
        if (llAngle > PI) {
            llAngle -= 2 * PI
        }
        if (llAngle < -PI) {
            llAngle += 2 * PI
        }
//        Log.i("KalmanMath", llAngle.toString())
        return Triple(xRobot, yRobot, thetaRobot)
    }
}