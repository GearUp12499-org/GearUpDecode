//package org.firstinspires.ftc.teamcode
//
//import android.util.Log
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.hardware.DcMotor
//import io.github.gearup12499.taskshark.FastScheduler
//import io.github.gearup12499.taskshark.prefabs.VirtualGroup
//import io.github.gearup12499.taskshark_android.TaskSharkAndroid
//import org.firstinspires.ftc.robotcore.external.Telemetry
//import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpoint2Driver
//import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
//import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
//import org.firstinspires.ftc.teamcode.systems.Combo
//import org.firstinspires.ftc.teamcode.systems.REmover
//import org.firstinspires.ftc.teamcode.systems.ShooterImpl
//import org.firstinspires.ftc.teamcode.tasks.SentinelTask
//import org.firstinspires.ftc.teamcode.tasks.compose
//import org.firstinspires.ftc.teamcode.utilities.StaticStore
//
//abstract class Auto3(private val red: Boolean) : LinearOpMode() {
//    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE
//
//    private lateinit var hw: CompBot2Hardware
//    private lateinit var shooter: ShooterImpl
//
//
//    override fun runOpMode() {
//        TaskSharkAndroid.setup()
//
//        hw = CompBot2Hardware(hardwareMap)
//        hw.slider.position = CompBot2Hardware.SLIDER_IN
//        hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_STOP_STOWED
//        hw.hood.position = 0.2756
//        hw.pinpoint.setPosition(poseSet.goalStart.asPose2D)
//
//        StaticStore.fallbackArtboard = if (red) Artboard.ARTBOARD_0 else Artboard.ARTBOARD_1
//        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
//
//        hw.turret.targetPosition = 0
//        hw.turret.mode = DcMotor.RunMode.RUN_TO_POSITION
//        hw.turret.power = 1.0
//
//        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
//        telemetry.update()
//        telemetry.addLine("<big><big>This is a " +
//                "<font color=\"${if (red) "#ff4040" else "#00ffff"}\"><strong>${if (red) "RED" else "BLUE"}</strong></font>" +
//                " auto</big></big>")
//        telemetry.update()
//
//        val sch = FastScheduler()
//        shooter = sch.add(ShooterImpl(hw))
//
//        val startFlag = sch.add(SentinelTask())
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
//        startFlag.then(VirtualGroup {
//            add(REmover.drive2Pose2(hw, poseSet.closeShoot))
//            add(shooter.setTargetAndWait(1300.0, 0.3))
//        })
//            .then(Combo.shoot(hw, shooter, 0.5))
////            .then(shooter.setTargetAsync(0.0))
////            .then(VirtualGroup {
////                val intake = add(Combo.intake(hw))
////                add(REmover.drive2Pose2(hw, poseSet.set3pos))
////                    .then(REmover.drive2Pose2(hw, poseSet.set3out))
////                    .then(VirtualGroup {
////                        add(REmover.drive2Pose2(hw, poseSet.farShoot))
////                        add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE1, 0.2))
////                    })
////                    .then(OneShot {
////                        intake.finish()
////                    })
////            })
////            .then(Combo.shoot(hw, shooter, 0.5))
//
//        while (opModeInInit()) sch.tick()
//
//        startFlag.finish()
//
//        while (opModeIsActive()) sch.tick()
//
//        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)
//        StaticStore.mark() // indicate to carry pinpoint into teleop in the next 30 seconds
//    }
//
////    private inner class Configurator : Task<Configurator>() {
////        private var rbt = false
////
////        override fun onTick(): Boolean {
////            if (opModeIsActive()) finish()
////
////            val rb = gamepad1.right_bumper
////            if (rb && !rbt) {
////                reconfigure(!skipExtra)
////            }
////
////            rbt = rb
////
////            return false // use finish() to kill this
////        }
////    }
//}