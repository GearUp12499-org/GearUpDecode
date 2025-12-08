package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;
import org.firstinspires.ftc.teamcode.systems.AprilTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Tune Cam Settings")
public class AutoTuneCamSettings extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private CompBotHardware hardware;
    //private Pose2D pose2D;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ExposureControl exposureControl;
    private GainControl gainControl;

    private int minExposure, maxExposure, currentExposure;
    private int minGain, maxGain, currentGain;

    private int finalExposure;
    private int finalGain;

    private List<String> detectionSequence = new ArrayList<>();
    private String motifOrder;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    @Override
    public void runOpMode() {
        hardware = new CompBotHardware(hardwareMap);
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

        // Set to chosen values before starting loop (given through above autoTune method)
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

            telemetry.addData("Detection Sequence: ", detectionSequence.toString());
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
                .setCameraPose(AprilTag.GSC_POSITION, AprilTag.GSC_ORIENTATION)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM)
            builder.setCamera(hardware.gsc);
        else
            builder.setCamera(BuiltinCameraDirection.BACK);

        builder.setCameraResolution(AprilTag.GSC_RESOLUTION);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void setupCameraControls() {
        if (visionPortal == null) return;

        telemetry.addLine("Checking camera capabilities...");
        telemetry.update();

        // Wait for the camera to start
        while (opModeInInit() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            sleep(20);

        // Retrieve camera controls
        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);

        // Setup exposure (manual mode)
        if (exposureControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
            exposureControl.setExposure(minExposure, TimeUnit.MILLISECONDS);
        }

        // Setup gain
        if (gainControl != null) {
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
            currentGain = 20;
            gainControl.setGain(currentGain);
        }

        telemetry.addLine("Camera ready & manual controls active");
        telemetry.update();
    }

    private void autoTuneExposureAndGain() {
        telemetry.addLine("Camera online — beginning auto-tune sequence...");
        telemetry.update();

        //Gain starts at 20 (look at above in the if statement)
        int exposureMin = 0;
        int exposureMax = 5;
        int gainMin = 0;
        int gainMax = 150;
        int exposureStep = 1;
        int gainStep = 10;

        boolean detectionFound = false;

        for (int e = exposureMin; e <= exposureMax; e += exposureStep) {
            exposureControl.setExposure(e, TimeUnit.MILLISECONDS);
            sleep(140);

            for (int g = gainMin; g <= gainMax; g += gainStep) {
                gainControl.setGain(g);
                sleep(120);
                //print for testing
                telemetry.addData("Testing", "Exposure=%d | Gain=%d", e, g);
                telemetry.update();

                boolean detected = checkAprilTagDetection(detectionSequence, e * 1000, g, "E/G");
                if (detected) {
                    telemetry.addData("Detection", "Found tag at Exposure=%d | Gain=%d ", e, g);
                    telemetry.update();

                    finalExposure = e;
                    finalGain = g;
                    detectionFound = true;

                    // Optionally test a few nearby points for stability
                    for (int fineE = Math.max(exposureMin, e - 1); fineE <= Math.min(exposureMax, e + 1); fineE++) {
                        for (int fineG = Math.max(gainMin, g - 15); fineG <=g ; fineG += 5) { //lower gain values work better in brighter conditions
                            exposureControl.setExposure(fineE, TimeUnit.MILLISECONDS);
                            gainControl.setGain(fineG);
                            sleep(120);
                            if(checkAprilTagDetection(detectionSequence, fineE * 1000, fineG, "Fine E/G")){
                                finalGain = fineG;
                                telemetry.addLine(motifOrder);
                                break;
                            }
                        }
                    }
                    break; // break gain loop
                }
            }

            if (detectionFound) break; // break exposure loop if already found
        }

        // Fallback if no detection
        if (!detectionFound) {
            finalExposure = 2;
            finalGain = 15;
            telemetry.addLine("No detections found — using fallback defaults");
            telemetry.addData("Final Exposure", finalExposure);
            telemetry.addData("Final Gain", finalGain);
            telemetry.update();
        }

        telemetry.addLine("Auto-tuning complete");
        telemetry.addData("Final Exposure", finalExposure);
        telemetry.addData("Final Gain", finalGain);
        telemetry.addData("Detection Sequence", detectionSequence.toString());
        telemetry.update();
    }

    private boolean checkAprilTagDetection(List<String> detectionSequence, int exp, int gain, String mode) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean detected = false;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.metadata.name.contains("Obelisk")) {
                detectionSequence.add(String.format("%s: %d, #d detected at ID %d |", mode, exp, gain, detection.id));
                if(detection.id == 21){
                    motifOrder = "GPP";
                } else if(detection.id == 22){
                    motifOrder = "PGP";
                } else{
                    motifOrder = "PPG"; //ID = 23
                }
                detected = true;
                break;
            }
        }
        if (!detected) {
            detectionSequence.add(String.format("%s: %d, %d no detection |", mode, exp, gain));
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

//        if (zoomControl != null) {
//            if (gamepad1.a) currentZoom = maxZoom;
//            if (gamepad1.b) currentZoom = minZoom;
//            if (gamepad1.dpad_right) currentZoom = Range.clip(currentZoom + 1, minZoom, maxZoom);
//            if (gamepad1.dpad_left) currentZoom = Range.clip(currentZoom - 1, minZoom, maxZoom);
//
//            zoomControl.setZoom(currentZoom);
//        }

        telemetry.addLine("\n---- Camera Settings ----");
        telemetry.addData("Exposure", "%d ms (%d–%d)", currentExposure, minExposure, maxExposure);
        telemetry.addData("Gain", "%d (%d–%d)", currentGain, minGain, maxGain);
//        telemetry.addData("Zoom", "%d (min=%d, max=%d)", currentZoom, minZoom, maxZoom);
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addData("Range (in)", detection.ftcPose.range);
                telemetry.addData("Bearing (deg)", detection.ftcPose.bearing);
//                double angleErr = Math.abs(Math.abs(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES))
//                        - pose2D.getHeading(AngleUnit.DEGREES));
//                telemetry.addData("Angle Error (Yaw)", angleErr);
                if (!detection.metadata.name.contains("Obelisk"))
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
