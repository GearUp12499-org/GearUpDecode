package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.gearup12499.taskshark.FastScheduler
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware
import org.firstinspires.ftc.teamcode.systems.AprilTag
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs

@TeleOp
class StandaloneReadAprilTag : LinearOpMode() {
    override fun runOpMode() {
        val hw = CompBotHardware(hardwareMap)
        val aprilTag = AprilTag(hw.gsc)
        val sch = FastScheduler()

        val aprilTagSetup = sch.add(aprilTag.setupAprilTag(0L, 70))

        while (opModeInInit()) {
            sch.tick()
            telemetry.addData("setup...", aprilTagSetup.getState())
            telemetry.update()
        }

        waitForStart()
        while (opModeIsActive()) {
            sch.tick()

            val detections: MutableList<AprilTagDetection> = aprilTag.aprilTagProcessor!!.detections
            telemetry.addData("# AprilTags Detected", detections.size)

            for (detection in detections) {
                if (detection.metadata != null && detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(
                        String.format(
                            "\n==== (ID %d) %s",
                            detection.id,
                            detection.metadata.name
                        )
                    )
                    telemetry.addData("Range (in)", detection.ftcPose.range)
                    telemetry.addData("Bearing (deg)", detection.ftcPose.bearing)
                    val angleErr: Double = abs(
                        abs(
                            detection.robotPose.orientation.getYaw(AngleUnit.DEGREES)
                        )
                    )
                    telemetry.addData("Angle Error (Yaw)", angleErr)
                    telemetry.addLine(
                        String.format(
                            "XYZ %6.1f %6.1f %6.1f (in)",
                            detection.robotPose.position.x,
                            detection.robotPose.position.y,
                            detection.robotPose.position.z
                        )
                    )
                }
            }

            telemetry.addLine("\nKey:")
            telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up)")
            telemetry.addLine("PRY = Pitch, Roll, Yaw (Rotations)")
            telemetry.update()
        }
    }
}