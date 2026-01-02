package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark.Task
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.REmover

@Autonomous
class REmoverTest : LinearOpMode() {
    override fun runOpMode() {
        TaskSharkAndroid.setup()
        val hw = CompBot2Hardware(hardwareMap)
        val sch = FastScheduler()

        hw.initMotion()

        hw.pinpoint.resetPosAndIMU()

        sch.add(object: Task.Anonymous() {
            override fun onTick(): Boolean {
                hw.pinpoint.update()
                return false
            }
        })
        sch.add(REmover.drive2Pose2(hw, REmover.RobotPose(0.0, 0.0, Math.toRadians(180.0))))

        while (opModeInInit()) {
            hw.pinpoint.update()
        }
        while (opModeIsActive()) {
            sch.tick()
        }
    }
}