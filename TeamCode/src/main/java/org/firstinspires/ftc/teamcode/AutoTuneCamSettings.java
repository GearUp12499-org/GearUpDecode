package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.*;

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
import org.firstinspires.ftc.teamcode.hardware.CompBotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Tune Cam Settings")
public class AutoTuneCamSettings extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private CompBotHardware hardware;
    private Pose2D pose2D;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ExposureControl exposureControl;
    private GainControl gainControl;

    private int minExposure, maxExposure, currentExposure;
    private int minGain, maxGain, currentGain;

    private int finalExposure;
    private int finalGain;

    private List<String> detectionSequence = new ArrayList<>();

    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 8.315, 7.73, 0);
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
        //autoTuneExposureAndGainMultiStage(false);
        //quickTuneExposureAndGain();

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

        builder.setCameraResolution(new Size(1600, 1200));
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
            currentGain = 15;
            gainControl.setGain(currentGain);
        }

        telemetry.addLine("Camera ready & manual controls active");
        telemetry.update();
    }

    private void autoTuneExposureAndGainMultiStage(boolean isBright) {
        // =====================
        // CONFIGURABLE VARIABLES
        // =====================
        int exposureMin = 0;      // minimum exposure in ms
        int exposureMax = 6;      // maximum exposure in ms
        int gainMin = 0;          // minimum gain
        int gainMax = 60;        // maximum gain
        int exposureStep = 1;     // coarse sweep step for exposure
        int gainStep = 20;        // coarse sweep step for gain
        int fineStep = 5;         // fine tuning step for gain
        int sleepTime = 120;      // ms to wait for camera to stabilize
        int defaultExposure = 3;  // fallback exposure
        int defaultGain = 15;     // fallback gain
        // =====================

        telemetry.addLine("Camera online — starting multi-stage auto-tune...");
        telemetry.update();

        // ---------------------
        // 1. COARSE SWEEP STAGE 1
        // ---------------------
        List<int[]> coarseHits = new ArrayList<>();
        for (int e = exposureMin; e <= exposureMax; e += exposureStep) {
            for (int g = gainMin; g <= gainMax; g += gainStep) {
                exposureControl.setExposure(e, TimeUnit.MILLISECONDS);
                gainControl.setGain(g);
                sleep(sleepTime);

                boolean detected = checkAprilTagDetection(detectionSequence, e*1000 + g, "Coarse 1");
                if (detected) {
                    coarseHits.add(new int[]{e, g});
                }
            }
        }

        // Choose the "best coarse point" (lowest exposure/gain detected, or fallback)
        int bestCoarseE, bestCoarseG;
        if (!coarseHits.isEmpty()) {
            bestCoarseE = coarseHits.get(0)[0];
            bestCoarseG = coarseHits.get(0)[1];
        } else {
            bestCoarseE = exposureMin;
            bestCoarseG = gainMin;
        }

        telemetry.addData("Coarse Stage 1", "Selected E=%d G=%d", bestCoarseE, bestCoarseG);
        telemetry.update();

        // ---------------------
        // 2. COARSE SWEEP STAGE 2 (around best coarse point)
        // ---------------------
        List<int[]> coarse2Hits = new ArrayList<>();
        int[] exposureRange = new int[]{Math.max(exposureMin, bestCoarseE-1), bestCoarseE, Math.min(exposureMax, bestCoarseE+1)};
        int[] gainRange = new int[]{Math.max(gainMin, bestCoarseG-10), bestCoarseG, Math.min(gainMax, bestCoarseG+10)};

        for (int e : exposureRange) {
            for (int g : gainRange) {
                exposureControl.setExposure(e, TimeUnit.MILLISECONDS);
                gainControl.setGain(g);
                sleep(sleepTime);

                boolean detected = checkAprilTagDetection(detectionSequence, e*1000 + g, "Coarse2");
                if (detected) {
                    coarse2Hits.add(new int[]{e, g});
                }
            }
        }

        // Pick best intermediate point
        int bestIntermediateE, bestIntermediateG;
        if (!coarse2Hits.isEmpty()) {
            bestIntermediateE = coarse2Hits.get(0)[0];
            bestIntermediateG = coarse2Hits.get(0)[1];
        } else {
            bestIntermediateE = bestCoarseE;
            bestIntermediateG = bestCoarseG;
        }

        telemetry.addData("Coarse Stage 2", "Selected E=%d G=%d", bestIntermediateE, bestIntermediateG);
        telemetry.update();

        // ---------------------
        // 3. FINE TUNING (around best intermediate point)
        // ---------------------
        boolean finalDetected = false;
        int[] fineExposureRange = new int[]{Math.max(exposureMin, bestIntermediateE-1), bestIntermediateE, Math.min(exposureMax, bestIntermediateE+1)};
        int[] fineGainRange = new int[]{
                Math.max(gainMin, bestIntermediateG - fineStep),
                bestIntermediateG,
                Math.min(gainMax, bestIntermediateG + fineStep)
        };

        for (int e : fineExposureRange) {
            for (int g : fineGainRange) {
                exposureControl.setExposure(e, TimeUnit.MILLISECONDS);
                gainControl.setGain(g);
                sleep(sleepTime);

                boolean detected = checkAprilTagDetection(detectionSequence, e*1000 + g, "FineTune");
                if (detected) {
                    finalExposure = e;
                    finalGain = g;
                    finalDetected = true;
                    break;
                }
            }
            if (finalDetected) break;
        }

        // ---------------------
        // 4. FALLBACK DEFAULT
        // ---------------------
        if (!finalDetected) {
            finalExposure = defaultExposure;
            finalGain = defaultGain;
        }

        telemetry.addLine("Multi-stage auto-tuning complete");
        telemetry.addData("Final Exposure", finalExposure);
        telemetry.addData("Final Gain", finalGain);
        telemetry.addData("Detection Sequence", detectionSequence.toString());
        telemetry.update();
    }

    private void quickTuneExposureAndGain() {
        // === USER-CONFIGURABLE VARIABLES ===
        int exposureStart = 0;      // Starting exposure value in milliseconds (lower = darker)
        int exposureEnd = 5;        // Maximum exposure to test (higher = brighter)
        int exposureStep = 1;       // How much to increase exposure per step

        int gainStart = 0;         // Starting analog gain (lower = less bright)
        int gainEnd = 60;          // Maximum gain to test
        int gainStep = 20;          // Step size for gain increase

        int stabilityChecks = 2;    // Number of frames per test so it gives the camera a chance to detect(use 1-3)
        int delayBetweenTests = 100; // Time in ms to wait after changing settings (100–150 ms)
        int detectionThreshold = 1; // Minimum number of detections to consider it successful

        // === INTERNAL VARIABLES ===
        int bestExposure = exposureStart;
        int bestGain = gainStart;
        boolean found = false;

        telemetry.addLine("Starting quick tuning...");
        telemetry.update();

        long startTime = System.currentTimeMillis();

        // Loop through exposure/gain combinations quickly
        outerLoop:
        for (int e = exposureStart; e <= exposureEnd; e += exposureStep) {
            exposureControl.setExposure(e, TimeUnit.MILLISECONDS);

            for (int g = gainStart; g <= gainEnd; g += gainStep) {
                gainControl.setGain(g);
                sleep(delayBetweenTests);

                int detections = 0;
                for (int i = 0; i < stabilityChecks; i++) {
                    if (checkAprilTagDetection(null, e * 1000 + g, "QuickTune")) {
                        detections++;
                        telemetry.addLine("Detected");
                    }
                }

                if (detections >= detectionThreshold) {
                    bestExposure = e;
                    bestGain = g;
                    found = true;
                    telemetry.addData("Found Detection", "Exposure: %d, Gain: %d", e, g);
                    telemetry.update();
                    break outerLoop; // Stop as soon as a detection works
                }

                // Safety check to ensure we don't exceed 2 seconds total
                if (System.currentTimeMillis() - startTime > 2000) {
                    telemetry.addLine("Quick tune timeout");
                    telemetry.update();
                    break outerLoop;
                }
            }
        }

        // Apply best found values
        exposureControl.setExposure(bestExposure, TimeUnit.MILLISECONDS);
        gainControl.setGain(bestGain);

        telemetry.addLine("Quick tuning complete");
        telemetry.addData("Best Exposure", bestExposure);
        telemetry.addData("Best Gain", bestGain);
        telemetry.addData("Detection Found", found);
        telemetry.update();
    }

    private void autoTuneExposureAndGain() {
        telemetry.addLine("Camera online — beginning auto-tune sequence...");
        telemetry.update();

        int exposureMin = 0;
        int exposureMax = 5;
        int gainMin = 0;
        int gainMax = 150;
        int exposureStep = 1;
        int gainStep = 10;

        boolean detectionFound = false;

        for (int e = exposureMin; e <= exposureMax; e += exposureStep) {
            exposureControl.setExposure(e, TimeUnit.MILLISECONDS);
            sleep(150);

            for (int g = gainMin; g <= gainMax; g += gainStep) {
                gainControl.setGain(g);
                sleep(120);

                telemetry.addData("Testing", "Exposure=%d | Gain=%d", e, g);
                telemetry.update();

                boolean detected = checkAprilTagDetection(detectionSequence, e * 1000 + g, "E/G");
                if (detected) {
                    telemetry.addData("Detection", "Found tag at Exposure=%d | Gain=%d", e, g);
                    telemetry.update();

                    finalExposure = e;
                    finalGain = g;
                    detectionFound = true;

                    // Optionally test a few nearby points for stability
                    for (int fineE = Math.max(exposureMin, e - 1); fineE <= Math.min(exposureMax, e + 1); fineE++) {
                        for (int fineG = Math.max(gainMin, g - 10); fineG <= Math.min(gainMax, g + 10); fineG += 5) {
                            exposureControl.setExposure(fineE, TimeUnit.MILLISECONDS);
                            gainControl.setGain(fineG);
                            sleep(120);
                            checkAprilTagDetection(detectionSequence, fineE * 1000 + fineG, "Fine E/G");
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
