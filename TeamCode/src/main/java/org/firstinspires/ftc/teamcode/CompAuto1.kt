package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.prefabs.OneShot
import io.github.gearup12499.taskshark.prefabs.Wait
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware.redFarStart
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
        hw.pinpoint.position = redFarStart.asPose2D
        hw.pinpoint.recalibrateIMU()

        val scheduler = FastScheduler()

        val shooter = scheduler.add(Shooter(hw.shooter1, hw.flipper))

        scheduler.add(REmover.drive2Pose(hw, CompBotHardware.shootPos))
            .then(shooter.setTargetAndWait(CompBotHardware.SHOOT_MIDRANGE))
            .then(OneShot {
                hw.flipper.position = CompBotHardware.FLIPPER_UP
            })
            .then(Wait.s(0.2))
            .then(OneShot {
                hw.flipper.position = CompBotHardware.FLIPPER_DOWN
            })
            .then(shooter.stopSoft())
            .then(Wait.s(3))

        waitForStart()
        while (opModeIsActive()) scheduler.tick()
    }
}