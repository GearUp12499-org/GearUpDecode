package org.firstinspires.ftc.teamcode

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.REmover
import org.openftc.easyopencv.OpenCvWebcam
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

@TeleOp
class DriveToPollen : LinearOpMode (){

    lateinit var hw: CompBot2Hardware

    override fun runOpMode() {

        val ftcDashboard = FtcDashboard.getInstance()

        val removerImpl = REmoverImpl()
        hw = CompBot2Hardware(hardwareMap)

        hw.pinpoint.setPosition(Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0))

        val pipe = 9

        waitForStart()

        hw.limelight.start()
        hw.limelight.setPollRateHz(100)
        hw.limelight.pipelineSwitch(pipe)

        //field coordinate system
        var tgtx = 0.0
        var tgty = 0.0
        var tgta = 0.0

        var targetExists = false
        var startTime = System.nanoTime()

        //limelight coordinate system
        var xLL = 0.0 //left right ???
        var yLL = 0.0 //up (+) down
        var zLL = 0.0 //depth : forward (+)

        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")

        ftcDashboard.startCameraStream(limelight, 30.0)

        while (opModeIsActive()) {


            hw.pinpoint.update()

            val currentPose = hw.pinpoint.position
            val xInch = currentPose.getX(DistanceUnit.INCH)
            val yInch = currentPose.getY(DistanceUnit.INCH)
            val thetaRad = currentPose.getHeading(AngleUnit.RADIANS)
            val velXInch = hw.pinpoint.getVelX(DistanceUnit.INCH)
            val velYInch = hw.pinpoint.getVelY(DistanceUnit.INCH)
            val angVelRad = hw.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            val speed = hypot(velXInch, velYInch)

            val result = hw.limelight.latestResult

            if (result != null){
                xLL = result.pythonOutput[1] //left-right
                yLL = result.pythonOutput[2] //up-down
                zLL = result.pythonOutput[3] //depth


                //make this return None/null when you don't find one


                if (yLL < 100 && xLL != 0.0 && yLL != 0.0 ){

                    startTime = System.nanoTime()
                    targetExists = true


//                    Log.i("X", xLL.toString())
//                    Log.i("Y", yLL.toString())
//                    Log.i("Z", zLL.toString())
//
//                    tgty = xLL * cos(thetaRad) - zLL * sin(thetaRad) + yInch
//                    tgtx = xLL * sin(thetaRad) + zLL * cos(thetaRad) + xInch
                    tgtx = xInch + zLL * cos(thetaRad) + xLL * sin(thetaRad)
                    tgty = yInch + zLL * sin(thetaRad) - xLL * cos(thetaRad)
                    tgta = atan2(-xLL, zLL) + thetaRad
//                    tgta = atan2(tgtx, tgty) + thetaRad

//                    removerImpl.init(
//                        xInch,
//                        yInch,
//                        thetaRad,
//                        REmover.RobotPose(tgtx, tgty, thetaRad),
//                        maxPower =0.5
//                    )
//

                    removerImpl.init(
                        xInch,
                        yInch,
                        thetaRad,
                        REmover.RobotPose(tgtx, tgty, tgta),
                        stopCond = StopConditions.Precision,
                        maxPower =1.0
                    )


                    Log.i("fieldX", tgtx.toString())
                    Log.i("fieldY", tgty.toString())
//                    Log.i("fieldZ", yLL.toString())
                    Log.i("offsetAngle", tgta.toString())


            }


                //|| (abs(removerImpl.deltaAll - removerImpl.prevDeltaAll)<0.1)
            if ((System.nanoTime() - startTime)>3e9 ) {
                startTime = System.nanoTime()
                tgtx = 0.0
                tgty = 0.0
                tgta = 0.0
                removerImpl.init(
                    xInch,
                    yInch,
                    thetaRad,
                    REmover.RobotPose(tgtx, tgty, tgta),
                    stopCond = StopConditions.Precision,
                    maxPower =0.5
                )
            }




            telemetry.addData("ppx", xInch)
            telemetry.addData("ppy", yInch)
            telemetry.addData("ppa", thetaRad)
//            telemetry.update()

            telemetry.addData("targetX", tgtx)
            telemetry.addData("targetY", tgty)
            telemetry.addData("targetA", tgta)
            telemetry.addData("depth", zLL)
            telemetry.addData("leftRight", xLL)
            telemetry.addData("updown", yLL)
            telemetry.update()
//
            if (targetExists) {
                removerImpl.tick(
                    xInch,
                    yInch,
                    thetaRad,
                    velXInch,
                    velYInch,
                    angVelRad,
                    speed,
                    hw
                )
            }



        }
    }}}

