package org.firstinspires.ftc.teamcode.variants

import android.media.audiofx.Virtualizer
import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.robot.Robot
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Scheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.Group
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark.prefabs.WaitUntil
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.AimingMachine
import org.firstinspires.ftc.teamcode.DriveMachine
import org.firstinspires.ftc.teamcode.GamepadState
import org.firstinspires.ftc.teamcode.IntakeShootMachine
import org.firstinspires.ftc.teamcode.IntakeShootMachine2
import org.firstinspires.ftc.teamcode.PoseSet
import org.firstinspires.ftc.teamcode.RobotState
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.tasks.WaitUntilContinuous

abstract class StateTeleop(private val red: Boolean) : LinearOpMode() {

    private lateinit var hw: CompBot2Hardware


    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE

//    val sch = FastScheduler()


    override fun runOpMode() {
        hw = CompBot2Hardware(hardwareMap)

        val driveMachine = DriveMachine(hw)
        val intakeShootMachine = IntakeShootMachine(hw)
//    val intakeShootMachine2 = IntakeShootMachine2(hw)
        val aimingMachine = AimingMachine(hw, poseSet)

        val robotState = RobotState(hw, red)
        val gamepadState = GamepadState(gamepad1, gamepad2)


        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        waitForStart()

        driveMachine.init(robotState)
        intakeShootMachine.init()
        aimingMachine.init()
        robotState.init()

        var loopStartTime = System.nanoTime().toDouble()
        val looptimeLimitMs = 45.0

        while (opModeIsActive()) {

            loopStartTime = System.nanoTime()/1e6

            val timeE = System.nanoTime()
            //DO ALL SENSOR READS HERE
            for (module in lynxModules) {
                module.clearBulkCache()
            }
            robotState.updateState()
            val timeF = System.nanoTime()
            Log.i("SensorRead", ((timeF-timeE)/1e6).toString())
            //get the current state of the gamepad
            gamepadState.update()

//            if (gamepadState.y2) {
//                sch.add(shoot())
//            }

//            sch.tick()
            val timeA = System.nanoTime()
            driveMachine.update(robotState, gamepadState)
            val timeB = System.nanoTime()
            Log.i("DriveMachine", ((timeB-timeA)/1e6).toString())
            intakeShootMachine.update(robotState, gamepadState)
            val timeC = System.nanoTime()
            Log.i("IntakeShootMachine", ((timeC-timeB)/1e6).toString())
            aimingMachine.update(robotState, gamepadState)
            val timeD = System.nanoTime()
            Log.i("AimingMachine", ((timeD-timeC)/1e6).toString())

            while ((System.nanoTime() / 1e6 - loopStartTime) < looptimeLimitMs) {
                sleep(1)
            }
//            loopStartTime = System.nanoTime()
            Log.i("TOTALTIME", ((System.nanoTime()-timeE)/1e6).toString())

        }
    }

//
//    fun shoot() = object : Group({}) {
//        init {
//            getScheduler()
//                .add(OneShot {
//                    intakeShootMachine2.intakeState = IntakeShootMachine2.IntakeState.INTAKING
//                    intakeShootMachine2.shooterStopState = IntakeShootMachine2.ShooterStopState.UP
//                    //add prism
//                })
//                .then(WaitUntilContinuous(0.15, max = 1.0) {
//                    !robotState.frontRamp && (robotState.colorBottomLeft < 110.0
//                            || robotState.colorBottomRight < 110.0)
//                })
//                .then(OneShot {
//                    intakeShootMachine2.flipperState = IntakeShootMachine2.FlipperState.UP
//                })
//                .then(Wait.ms(400))
//                .then(OneShot {
//                    intakeShootMachine2.intakeState = IntakeShootMachine2.IntakeState.REVERSING
//                    intakeShootMachine2.flipperState = IntakeShootMachine2.FlipperState.DOWN
//                })
//                .then(Wait.ms(500))
//                .then(OneShot {
//                    intakeShootMachine2.intakeState = IntakeShootMachine2.IntakeState.IDLE
//                    intakeShootMachine2.shooterStopState = IntakeShootMachine2.ShooterStopState.DOWN
//                    //prism
//                })
//        }
//        )

    }
//}


//}


