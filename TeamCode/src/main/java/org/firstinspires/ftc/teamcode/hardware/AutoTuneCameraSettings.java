package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.hardware.DumbledoreHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Tune Cam Settings")
public class AutoTuneCameraSettings extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private DumbledoreHardware hardware;
    private Pose2D pose2D;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private PtzControl zoomControl;
    private ExposureControl exposureControl;
    private GainControl gainControl;

    private int minExposure, maxExposure, currentExposure;
    private int minGain, maxGain, currentGain;
    private int minZoom, maxZoom, currentZoom = 1;

    private int finalExposure;
    private int finalGain;

    private List<String> detectionSequence = new ArrayList<>();

    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 8.315, 7.73, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    @Override
    public void runOpMode() {
      hardware = new DumbledoreHardware(hardwareMap);
//        hardware.PinPoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
//        hardware.PinPoint.setOffsets(96, 24, DistanceUnit.MM);
//        hardware.PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        hardware.PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD);
//
//        hardware.PinPoint.resetPosAndIMU();
//        hardware.PinPoint.recalibrateIMU();

        initAprilTag();
        setupCameraControls();

        // Run auto-tuning before starting main loop
        autoTuneExposureAndGain();

        // Set to chosen values before starting loop
        currentExposure = finalExposure;
        currentGain = finalGain;
        if (exposureControl != null) exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
        if (gainControl != null) gainControl.setGain(currentGain);

        telemetry.addData("Init", "Complete. Use bumpers/triggers/D-pad to adjust camera settings.");
        telemetry.addData("Final Exposure", currentExposure);
        telemetry.addData("Final Gain", currentGain);
        telemetry.addData("Detection Sequence", detectionSequence.toString());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
//            hardware.PinPoint.update();
//            pose2D = hardware.PinPoint.getPosition();

//            telemetry.addData("X (in)", pose2D.getX(DistanceUnit.INCH));
//            telemetry.addData("Y (in)", pose2D.getY(DistanceUnit.INCH));
//            telemetry.addData("Heading (deg)", pose2D.getHeading(AngleUnit.DEGREES));

            telemetryAprilTag();
            handleCameraControls();

            telemetry.addData("Detection Sequence", detectionSequence.toString());
            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM)
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        else
            builder.setCamera(BuiltinCameraDirection.BACK);

        builder.setCameraResolution(new Size(1280, 960));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void setupCameraControls() {
        if (visionPortal == null) return;

        telemetry.addLine("Checking camera capabilities...");
        telemetry.update();

        while (opModeInInit() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        zoomControl = visionPortal.getCameraControl(PtzControl.class);
        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);

        if (exposureControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
//            currentExposure = Range.clip(10, minExposure, maxExposure);
//            exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
            exposureControl.setExposure(minExposure, TimeUnit.MILLISECONDS);
        }

        if (gainControl != null) {
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
            currentGain = 15; //changed for now
            gainControl.setGain(currentGain);
        }

//        if (zoomControl != null) {
//            minZoom = (int) zoomControl.getMinZoom();
//            maxZoom = (int) zoomControl.getMaxZoom();
//            currentZoom = (int) zoomControl.getZoom();
//        }

//        telemetry.addData("Exposure Control", (exposureControl != null) ? "Supported" : "Not Supported");
//        telemetry.addData("Gain Control", (gainControl != null) ? "Supported" : "Not Supported");
//        telemetry.addData("Zoom Control", (zoomControl != null) ? "Supported" : "Not Supported");
//        sleep(250);
        telemetry.update();
    }

    private void autoTuneExposureAndGain() {
        boolean detected = false;

        telemetry.addLine("Starting auto exposure tuning...");
        telemetry.update();

        if (exposureControl != null) {
            for (int e = 1; e <= 4; e++) {
                telemetry.addData("Testing Exposure", e);
                telemetry.update();
                exposureControl.setExposure(e, TimeUnit.MILLISECONDS);
                sleep(300);
                if (checkAprilTagDetection(detectionSequence, e, "Exposure")) {
                    finalExposure = e;
//                    detected = true; We still want to find the best gain
                    break;
                }
            }

//            if (!detected) {
//                for (int e = 8; e <= 10; e += 2) {
//                    telemetry.addData("Testing Exposure", e);
//                    telemetry.update();
//                    exposureControl.setExposure(e, TimeUnit.MILLISECONDS);
//                    sleep(500);
//                    if (checkAprilTagDetection(detectionSequence, e, "Exposure")) {
//                        finalExposure = e;
//                        finalGain = currentGain;
//                        detected = true;
//                        break;
//                    }
//                }
//            }
        }

        if (detected) {
            telemetry.addLine("Detection achieved during exposure tuning!");
            telemetry.update();
            return;
        }

        if (gainControl != null) {
            telemetry.addLine("Starting auto gain tuning...");
            telemetry.update();

            if (exposureControl != null) exposureControl.setExposure(finalExposure, TimeUnit.MILLISECONDS);

            int gainStart = 70;

            for (int g = gainStart; g >= minGain; g -= 10) {
                telemetry.addData("Testing Gain", g);
                telemetry.update();
                gainControl.setGain(g);
                sleep(250);
                if (checkAprilTagDetection(detectionSequence, g, "Gain")) {
                    if(g <= 20){
                        finalGain = 20;
                    } else if(g<=50){
                        finalGain = g - 10;
                    } else{
                        finalGain = g - 20;
                    }
                    detected = true;
                    break;
                }
            }
        }

        if (!detected) {
            finalExposure = 3;
            finalGain = 13;
        }

        telemetry.addLine("Auto tuning complete!");
        telemetry.addData("Final Exposure", finalExposure);
        telemetry.addData("Final Gain", finalGain);
        telemetry.addData("Detection Sequence", detectionSequence.toString());
        telemetry.update();
    }

    private boolean checkAprilTagDetection(List<String> detectionSequence, int value, String mode) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean detected = false;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.metadata.name.contains("Obelisk")) {
                detectionSequence.add(String.format("%s: %d detected at ID %d", mode, value, detection.id));
                detected = true;
                break;
            }
        }
        if (!detected) {
            detectionSequence.add(String.format("%s: %d no detection", mode, value));
        }
        return detected;
    }

    private void handleCameraControls() {
        if (exposureControl != null && (gamepad1.left_bumper || gamepad1.left_trigger > 0.25)) {
            if (gamepad1.left_bumper) currentExposure += 1;
            else if (gamepad1.left_trigger > 0.25) currentExposure -= 1;
            sleep(100);
//            currentExposure = Range.clip(currentExposure, minExposure, maxExposure);
            exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
        }

        if (gainControl != null && (gamepad1.right_bumper || gamepad1.right_trigger > 0.25)) {
            if (gamepad1.right_bumper) currentGain += 1;
            else if (gamepad1.right_trigger > 0.25) currentGain -= 1;
            sleep(100);
//            currentGain = Range.clip(currentGain, minGain, maxGain);
            gainControl.setGain(currentGain);
        }

        if (zoomControl != null) {
            if (gamepad1.a) currentZoom = maxZoom;
            if (gamepad1.b) currentZoom = minZoom;
            if (gamepad1.dpad_right) currentZoom = Range.clip(currentZoom + 1, minZoom, maxZoom);
            if (gamepad1.dpad_left) currentZoom = Range.clip(currentZoom - 1, minZoom, maxZoom);

            zoomControl.setZoom(currentZoom);
        }

        telemetry.addLine("\n---- Camera Settings ----");
        telemetry.addData("Exposure", "%d ms (%d–%d)", currentExposure, minExposure, maxExposure);
        telemetry.addData("Gain", "%d (%d–%d)", currentGain, minGain, maxGain);
        telemetry.addData("Zoom", "%d (min=%d, max=%d)", currentZoom, minZoom, maxZoom);
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addData("Range (in)", detection.ftcPose.range);
                telemetry.addData("Bearing (deg)", detection.ftcPose.bearing);
                double angleErr = Math.abs(Math.abs(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES))
                        - pose2D.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Angle Error (Yaw)", angleErr);
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (in)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
            }
        }

        telemetry.addLine("\nKey:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up)");
        telemetry.addLine("PRY = Pitch, Roll, Yaw (Rotations)");
    }
}
