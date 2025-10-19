/* Copyright (c) 2024 Dryw Wade. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.DumbledoreHardware;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpoint2Driver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of AprilTag based localization.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will be used to compute the robot's location and orientation.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the robot, relative to the field origin.
 * This information is provided in the "robotPose" member of the returned "detection".
 *
 * To learn about the Field Coordinate System that is defined for FTC (and used by this OpMode), see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Concept: AprilTag Localization", group = "Concept")
public class ConceptAprilTagLocalization extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     * x-axis = pitch
     * y-axis = yaw
     * z-axis = roll
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 7.75, 0, 0); //y = 7.75,
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private PtzControl zoomControl;
    DumbledoreHardware hardware;
    Pose2D pose2D;
    // Manual camera control values (you can change these before running)
    private static int manualExposureMs = 1; // exposure in milliseconds
    private static int manualGain = 100;       // camera gain (typically 0–255)

    @Override
    public void runOpMode() {
        hardware = new DumbledoreHardware(hardwareMap);

        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES,0));
        hardware.PinPoint.setOffsets(96,24, DistanceUnit.MM);
        hardware.PinPoint.setEncoderResolution(GoBildaPinpoint2Driver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        hardware.PinPoint.setEncoderDirections(GoBildaPinpoint2Driver.EncoderDirection.FORWARD, GoBildaPinpoint2Driver.EncoderDirection.FORWARD);

        hardware.PinPoint.resetPosAndIMU();
        hardware.PinPoint.recalibrateIMU();

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("Look up! DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pose2D = hardware.PinPoint.getPosition();
//            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
//            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
//            telemetry.addData("Heading angle (DEGREES)\n", pose2D.getHeading(AngleUnit.DEGREES));

            hardware.PinPoint.update();

            telemetryAprilTag();
            if (USE_WEBCAM) {
                zoomControl = visionPortal.getCameraControl(PtzControl.class);
            }
            if (zoomControl != null) {
                if (gamepad1.a) {
                    zoomControl.setZoom(zoomControl.getMaxZoom());
                    telemetry.addData("Zoom", "Set to Max");
                }
                if (gamepad1.b) {
                    zoomControl.setZoom(zoomControl.getMinZoom());
                    telemetry.addData("Zoom", "Set to Min");
                }
            }

            // === Manual Exposure and Gain Control ===
            try {
                // Get camera controls
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                ExposureControl exposureControl = visionPortal.getCameraControl(
                        ExposureControl.class);
                // === Controller input for tuning ===
                if (gamepad1.a) { // increase exposure
                    manualExposureMs = Math.min(30, manualExposureMs + 1);
                    sleep(150);
                } else if (gamepad1.b) { // decrease exposure
                    manualExposureMs = Math.max(1, manualExposureMs - 1);
                    sleep(150);
                }

                if (gamepad1.x) { // increase gain
                    manualGain = Math.min(255, manualGain + 5);
                    sleep(150);
                } else if (gamepad1.y) { // decrease gain
                    manualGain = Math.max(1, manualGain - 5);
                    sleep(150);
                }
                // Set exposure (manual mode)
                if (exposureControl != null) {
                    exposureControl.setMode(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.Mode.Manual);
                    exposureControl.setExposure(manualExposureMs, TimeUnit.MILLISECONDS);
                }

                // Set gain
                if (gainControl != null) {
                    gainControl.setGain(manualGain);
                }

                telemetry.addLine("Manual camera settings applied:");
                telemetry.addData("Exposure (ms)", manualExposureMs);
                telemetry.addData("Gain", manualGain);
                telemetry.update();

            } catch (Exception e) {
                telemetry.addLine("Failed to set manual exposure/gain!");
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
            }

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(960, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addLine("\n==== APRILTAG DETECTION SUMMARY ====");
        telemetry.addData("Number of Tags Detected", detections.size());

        // If no tags are detected, just show the count
        if (detections.isEmpty()) {
            telemetry.addLine("No AprilTags currently detected.");
        }

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine("\n-------------------------------");
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Tag Name", detection.metadata.name);

                // === Pose Information ===
                telemetry.addLine("Pose (Relative to Field Origin):");
                telemetry.addData("X (Right, in)", String.format("%.2f", detection.robotPose.getPosition().x));
                telemetry.addData("Y (Forward, in)", String.format("%.2f", detection.robotPose.getPosition().y));
                telemetry.addData("Z (Up, in)", String.format("%.2f", detection.robotPose.getPosition().z));

                double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                double pitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                double roll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);

                telemetry.addData("Yaw (deg)", String.format("%.2f", yaw));
                telemetry.addData("Pitch (deg)", String.format("%.2f", pitch));
                telemetry.addData("Roll (deg)", String.format("%.2f", roll));

                // === Bearing, Range, and Errors ===
                double bearing = detection.ftcPose.bearing; // degrees offset from camera center
                double range = detection.ftcPose.range;     // distance from camera to tag center (inches)

                telemetry.addLine("\nBearing and Range:");
                telemetry.addData("Bearing (deg)", String.format("%.2f", bearing));
                telemetry.addData("Range (in)", String.format("%.2f", range));

                double expectedRange = 24.0; // Example: known test distance in inches
                double expectedBearing = 0.0; // Expected straight ahead

                double rangeError = range - expectedRange;
                double bearingError = bearing - expectedBearing;

                telemetry.addLine("---- Errors & Offsets ----");
                telemetry.addData("Range Error (in)", String.format("%.2f", rangeError));
                telemetry.addData("Bearing Error (deg)", String.format("%.2f", bearingError));

                // Yaw offset relative to robot heading
                double robotYaw = pose2D.getHeading(AngleUnit.DEGREES);
                double yawOffset = yaw - robotYaw;
                telemetry.addData("Yaw Offset (deg)", String.format("%.2f", yawOffset));
            }
        } // end method telemetryAprilTag()

        // === Current Camera Settings (for manual tuning) ===
        try {
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            if (exposureControl != null && gainControl != null) {
                telemetry.addLine("\nCAMERA SETTINGS");
                telemetry.addData("Exposure (ms)", manualExposureMs);
                telemetry.addData("Gain", manualGain);
            }
        } catch (Exception e) {
            telemetry.addLine("\nCAMERA SETTINGS UNAVAILABLE");
        }

        // === Telemetry Key ===
        telemetry.addLine("\n---- TELEMETRY KEY ----");
        telemetry.addLine("Tag ID = Detected AprilTag identifier");
        telemetry.addLine("X/Y/Z = Position of tag relative to field origin (inches)");
        telemetry.addLine("Yaw/Pitch/Roll = Tag orientation relative to field");
        telemetry.addLine("Bearing = Angle offset from camera center to tag center");
        telemetry.addLine("Range = Distance from camera to tag center");
        telemetry.addLine("Yaw Offset = Difference between tag yaw and robot heading");
        telemetry.addLine("Range/Bearing Error = Difference from known test values");
    }

}   // end class
