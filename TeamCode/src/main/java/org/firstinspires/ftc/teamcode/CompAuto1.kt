package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.api.LogOutlet
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.redFarStartPose
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver.EncoderDirection
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD
import org.firstinspires.ftc.teamcode.systems.REmover
import org.firstinspires.ftc.teamcode.systems.Shooter

@Autonomous
class CompAuto1 : LinearOpMode() {
    lateinit var hw: CompBotHardware
    override fun runOpMode() {
        TaskSharkAndroid.setup()
        hw = CompBotHardware(hardwareMap)

        hw.pinpoint.setOffsets(-3.9, -3.875, DistanceUnit.INCH)
        hw.pinpoint.setEncoderResolution(goBILDA_4_BAR_POD)
        hw.pinpoint.setEncoderDirections(
            EncoderDirection.REVERSED,
            EncoderDirection.FORWARD
        )
        hw.pinpoint.position = redFarStartPose
        hw.pinpoint.recalibrateIMU()

        val scheduler = FastScheduler()

        val remover = scheduler.add(REmover(hw))
        val shooter = scheduler.add(Shooter(hw.shooter1, hw.flipper))

        scheduler.add(remover.drive2Pose(CompBotHardware.shootPos))
            .then(OneShot {
                shooter.targetVelocity = 1200.0
            })
            .then(shooter.waitForTargetHold(0.5))
            .then(OneShot {
                hw.flipper.position = CompBotHardware.FLIPPER_UP
            })
            .then(Wait.s(0.2))
            .then(OneShot {
                hw.flipper.position = CompBotHardware.FLIPPER_DOWN
            })
            .then(Wait.s(3))

        waitForStart()
        while (opModeIsActive()) scheduler.tick()
    }
}