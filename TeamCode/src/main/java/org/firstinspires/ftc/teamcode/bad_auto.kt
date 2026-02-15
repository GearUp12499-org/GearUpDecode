//package org.firstinspires.ftc.teamcode
//
//import android.util.Log
//import com.qualcomm.robotcore.eventloop.opmode.Disabled
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import com.qualcomm.robotcore.hardware.DcMotor
//import io.github.gearup12499.taskshark.FastScheduler
//import io.github.gearup12499.taskshark.ITask
//import io.github.gearup12499.taskshark.Scheduler
//import io.github.gearup12499.taskshark.Task
//import io.github.gearup12499.taskshark.prefabs.Group
//import io.github.gearup12499.taskshark.prefabs.OneShot
//import io.github.gearup12499.taskshark.prefabs.WaitUntil
//import io.github.gearup12499.taskshark_android.TaskSharkAndroid
//import org.firstinspires.ftc.robotcore.external.Telemetry
//import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver
//import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
//import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
//import org.firstinspires.ftc.teamcode.systems.Combo
//import org.firstinspires.ftc.teamcode.systems.ShooterImpl
//import org.firstinspires.ftc.teamcode.tasks.SentinelTask
//import org.firstinspires.ftc.teamcode.tasks.compose
//import org.firstinspires.ftc.teamcode.utilities.StaticStore
//
//@Disabled
//@TeleOp
//class bad_auto() : LinearOpMode() {
//    private val red = true
//    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE
//
//    private lateinit var hw: CompBot2Hardware
//    private lateinit var shooter: ShooterImpl
//
//    private var altnStart = false
//    private var confTask: ITask<*>? = null
//
//    fun reconfigure(altn: Boolean, prismBroken: Boolean, sch: Scheduler) {
//        altnStart = altn
//        StaticStore.prismBroken = prismBroken
//        hw.refreshPrismState()
//
//        confTask?.stop()
//        confTask = sch.add(Group {
//            it.add(OneShot {
//                hw.pinpoint.recalibrateIMU()
//            }).then(WaitUntil {
//                hw.pinpoint.deviceStatus == GoBildaPinpoint2Driver.DeviceStatus.READY
//            }).then(OneShot {
//                hw.pinpoint.setPosition(if (altn) poseSet.goalStart.asPose2D else poseSet.farStart.asPose2D)
//            })
//        })
//
//
//        if (prismBroken) {
//            telemetry.addLine("PRISM IS DISABLED")
//            telemetry.addLine()
//        }
//
//        telemetry.addLine("<big><big>Auto Setup</big></big>")
//        telemetry.addLine("Position: <strong>${if (altn) "GOAL (ALTERNATE)" else "FAR (MAIN)"}</strong>")
//        telemetry.addLine("Press 1/RB to change")
//        telemetry.addLine("Press 1/X to toggle Prism")
//        telemetry.update()
//    }
//
//    override fun runOpMode() {
//        TaskSharkAndroid.setup()
//
//        hw = CompBot2Hardware(hardwareMap)
//        hw.slider.position = CompBot2Hardware.SLIDER_IN
//        hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_BALL_STOP
//        hw.hood.position = CompBot2Hardware.HOOD_50
//
//        hw.turret.targetPosition = 0
//        hw.turret.mode = DcMotor.RunMode.RUN_TO_POSITION
//        hw.turret.power = 1.0
//
//        StaticStore.fallbackArtboard = if (red) Artboard.ARTBOARD_0 else Artboard.ARTBOARD_1
//        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
//
//        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
//        telemetry.update()
//
//        val sch = FastScheduler()
//        reconfigure(false, StaticStore.prismBroken, sch)
//
//        sch.add(Configurator())
//
//        val startFlag = sch.add(SentinelTask())
//        shooter = sch.add(ShooterImpl(hw))
//
//        sch.add(compose {
//            var state: GoBildaPinpoint2Driver.DeviceStatus? = null
//            onTick {
//                val stateNew = hw.pinpoint.deviceStatus
//                if (stateNew != state) {
//                    Log.w("Pinpoint", "state changed $state -> $stateNew")
//                    state = stateNew
//                }
//                hw.pinpoint.update()
//                false
//            }
//        })
//
//        startFlag.then(Combo.intakeBox(hw))
//            .then(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_MID_RANGE))
////            .then(Combo.shootBoxFirst(hw, shooter))
//            .then(Combo.shootBoxMiddle(hw, shooter))
////            .then(Combo.shootBoxLast(hw, shooter))
//
//        while (opModeInInit()) sch.tick()
//
//        startFlag.finish()
//
//        while (opModeIsActive()) sch.tick()
//
//        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
////        StaticStore.mark() // indicate to carry pinpoint into teleop in the next 30 seconds
//    }
//
//    private inner class Configurator : Task<Configurator>() {
//        private var rbt = false
//        private var xt = false
//
//        override fun onTick(): Boolean {
//            if (opModeIsActive()) finish()
//
//            val rb = gamepad1.right_bumper
//            val x = gamepad1.x
//            if (rb && !rbt) {
//                reconfigure(!altnStart, StaticStore.prismBroken, scheduler!!)
//            }
//            if (x && !xt) {
//                reconfigure(altnStart, !StaticStore.prismBroken, scheduler!!)
//            }
//
//            rbt = rb
//            xt = x
//
//            return false // use finish() to kill this
//        }
//    }
//}