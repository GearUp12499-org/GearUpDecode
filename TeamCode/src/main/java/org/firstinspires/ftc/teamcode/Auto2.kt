package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.VirtualGroup
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.teamcode.drivers.GoBildaPrismDriver.Artboard
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.Combo
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.ShooterImpl
import org.firstinspires.ftc.teamcode.utilities.StaticStore

abstract class Auto2(private val red: Boolean) : LinearOpMode() {
    private val poseSet = if (red) PoseSet.RED else PoseSet.BLUE

    private lateinit var hw: CompBot2Hardware
    private lateinit var shooter: ShooterImpl

    override fun runOpMode() {
        TaskSharkAndroid.setup()

        hw = CompBot2Hardware(hardwareMap)
        hw.dropDown.position = CompBot2Hardware.DROP_DOWN_BOTTOM
        hw.slider.position = CompBot2Hardware.SLIDER_IN
        hw.bottomBallStop.position = CompBot2Hardware.BOTTOM_BALL_STOP
        hw.hood.position = CompBot2Hardware.HOOD_UP
        hw.pinpoint.setPosition(poseSet.farStart.asPose2D)

        StaticStore.fallbackArtboard = if (red) Artboard.ARTBOARD_0 else Artboard.ARTBOARD_1
        hw.prism.loadAnimationsFromArtboard(StaticStore.fallbackArtboard)

        val sch = FastScheduler()
        shooter = sch.add(ShooterImpl(hw))

        sch.add(object : Task.Anonymous() {
            override fun onTick(): Boolean {
                hw.pinpoint.update()
                return false
            }
        })

        sch.add(VirtualGroup {
            add(REmover.drive2Pose2(hw, poseSet.farShoot))
            add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.3))
        })
            .then(Combo.shoot(hw, 0.5))
            .then(shooter.setTargetAsync(0.0))
            .then(VirtualGroup {
                val intake = add(Combo.intake(hw))
                add(REmover.drive2Pose2(hw, poseSet.set3pos))
                    .then(REmover.drive2Pose2(hw, poseSet.set3out))
                    .then(VirtualGroup {
                        add(REmover.drive2Pose2(hw, poseSet.farShoot))
                        add(shooter.setTargetAndWait(CompBot2Hardware.SHOOT_FAR_RANGE, 0.2))
                    })
                    .then(OneShot {
                        intake.finish()
                    })
            })
            .then(Combo.shoot(hw, 0.5))
            .then(REmover.drive2Pose2(hw, poseSet.set2pos))

        waitForStart()
        while (opModeIsActive()) sch.tick()

        StaticStore.mark() // indicate to carry pinpoint into teleop in the next 30 seconds
    }
}