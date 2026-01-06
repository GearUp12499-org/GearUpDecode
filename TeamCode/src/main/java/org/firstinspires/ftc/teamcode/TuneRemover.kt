package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import io.github.gearup12499.taskshark_android.TaskSharkAndroid
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.hardware.CompBot2Hardware
import org.firstinspires.ftc.teamcode.systems.REmover

@TeleOp
class TuneRemover : LinearOpMode() {

    private lateinit var hw: CompBot2Hardware
    override fun runOpMode() {
        TaskSharkAndroid.setup()
        val sch = FastScheduler()
        hw = CompBot2Hardware(hardwareMap)
//        hw.pinpoint.resetPosAndIMU()
        waitForStart()

        hw.pinpoint.setPosition(
            Pose2D(
                DistanceUnit.INCH,
                -63.375,
                -17.25,
                AngleUnit.RADIANS,
                -Math.PI
            )
        )

        var wasA = false;
        var wasB = false;
        var wasX = false;
        var wasY = false;

        while (opModeIsActive()) {
            sch.tick()
            hw.pinpoint.update()

            if (gamepad1.a && !wasA) {
                sch.add(REmover.drive2Pose2(hw, REmover.RobotPose(48.0, 0.0, 0.0)))
            }
            else if (gamepad1.b && !wasB) {
                sch.add(REmover.drive2Pose2(hw, REmover.RobotPose(0.0,-48.0,0.0)))
            }
            else if (gamepad1.x && !wasX) {
                sch.add(REmover.drive2Pose2(hw, REmover.RobotPose(0.0,0.0,Math.PI)))
            }
            else if (gamepad1.y && !wasY) {
                sch.add(REmover.drive2Pose2(hw, REmover.RobotPose(24.0,-24.0,3*Math.PI/4)))

            }


            wasA = gamepad1.a
            wasB = gamepad1.b
            wasX = gamepad1.x
            wasY = gamepad1.y
        }
    }}