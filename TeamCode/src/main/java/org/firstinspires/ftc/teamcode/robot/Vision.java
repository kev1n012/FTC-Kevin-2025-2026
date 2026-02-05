package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

//  THIS IS TESTING AND WILL NOT BE USED IN THE COMPETITION


public class Vision {

    // Hardware
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    // Camera config
    private static final int DEFAULT_EXPOSURE_MS = 6;
    private static final int DEFAULT_GAIN = 250;

    // Auto-aim config
    private static final double AUTO_AIM_GAIN = 0.015;
    private static final double AUTO_AIM_DEADZONE = 2.0;

    // Lookup table: each entry is one row of [tagId, minDist, maxDist, power, targetRPM]
    private final List<LookupEntry> lookupTable = new ArrayList<>();

    // ===================================================================
    // INNER CLASSES
    // ===================================================================

    /** One row in the lookup table */
    private static class LookupEntry {
        final int tagId;
        final double minDist, maxDist;
        final double power;
        final int targetRPM;

        LookupEntry(int tagId, double minDist, double maxDist, double power, int targetRPM) {
            this.tagId = tagId;
            this.minDist = minDist;
            this.maxDist = maxDist;
            this.power = power;
            this.targetRPM = targetRPM;
        }
    }

    /** Returned to the caller - contains everything you need */
    public static class DetectionResult {
        public final int tagId;
        public final double range;          // inches
        public final double bearing;        // degrees
        public final double power;          // from lookup table, -1 if not found
        public final int targetRPM;         // from lookup table, -1 if not found
        public final boolean hasSettings;   // whether lookup table matched

        public DetectionResult(int tagId, double range, double bearing, double power, int targetRPM) {
            this.tagId = tagId;
            this.range = range;
            this.bearing = bearing;
            this.power = power;
            this.targetRPM = targetRPM;
            this.hasSettings = (power >= 0);
        }
    }

    // ===================================================================
    // CONSTRUCTOR
    // ===================================================================

    public Vision(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        setManualExposure(DEFAULT_EXPOSURE_MS, DEFAULT_GAIN);

        initializeLookupTable();
    }

    // ===================================================================
    // LOOKUP TABLE SETUP - edit initializeLookupTable() with your values
    // ===================================================================

    /**
     * Add your tested distance/power/RPM values here per tag ID.
     * Distances are in inches. Tune these based on actual testing!
     */
    private void initializeLookupTable() {
        // Goal Blue
        addLookupEntry(20, 90,   150,  0.55, 2300);
        addLookupEntry(20, 150,  190,  0.60, 1500);
        addLookupEntry(20, 190,  210, 0.65, 2100);
        addLookupEntry(20, 210, 220, 0.85, 3900);

        // Goal Red
        addLookupEntry(24, 0,   50,  0.60, 3000);
        addLookupEntry(24, 50,  80,  0.70, 3500);
        addLookupEntry(24, 80,  120, 0.80, 3900);
        addLookupEntry(24, 120, 200, 0.85, 4100);

    }

    /** Add a single row to the lookup table */
    public void addLookupEntry(int tagId, double minDist, double maxDist, double power, int targetRPM) {
        lookupTable.add(new LookupEntry(tagId, minDist, maxDist, power, targetRPM));
    }

    // ===================================================================
    // CORE LOGIC - plain methods, usable in TeleOp loop directly
    // ===================================================================

    /** Get the first tag with valid metadata */
    public AprilTagDetection getFirstDetection() {
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null) return d;
        }
        return null;
    }

    /** Get a specific tag by ID */
    public AprilTagDetection getDetectionById(int tagId) {
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null && d.id == tagId) return d;
        }
        return null;
    }

    /** Get all detections */
    public List<AprilTagDetection> getAllDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Get a DetectionResult with lookup table settings already applied.
     * Returns null if no tag is visible.
     */
    public DetectionResult getDetectionResult() {
        AprilTagDetection tag = getFirstDetection();
        return buildResult(tag);
    }

    /**
     * Get a DetectionResult for a specific tag ID with lookup table settings.
     * Returns null if the tag isn't visible.
     */
    public DetectionResult getDetectionResultById(int tagId) {
        AprilTagDetection tag = getDetectionById(tagId);
        return buildResult(tag);
    }

    /**
     * Calculate rotation power needed to center on a target bearing.
     * Use this directly in your TeleOp drive loop.
     */
    public double getAutoAimRotation(double bearing) {
        if (Math.abs(bearing) <= AUTO_AIM_DEADZONE) return 0;
        return -bearing * AUTO_AIM_GAIN;
    }

    /** Overload with custom gain and deadzone */
    public double getAutoAimRotation(double bearing, double gain, double deadzone) {
        if (Math.abs(bearing) <= deadzone) return 0;
        return -bearing * gain;
    }

    // ===================================================================
    // ACTIONS - thin wrappers for autonomous use with RoadRunner
    // ===================================================================

    /**
     * Blocks until a tag is detected or timeout is reached.
     * Useful at the start of an autonomous before you need to aim.
     */
    public Action WaitForDetectionAction(double timeoutSeconds) {
        return new Action() {
            private long startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) startTime = System.nanoTime();
                double elapsed = (System.nanoTime() - startTime) / 1e9;

                if (elapsed > timeoutSeconds) {
                    packet.put("Vision", "Timeout - no tag found");
                    return false;
                }

                AprilTagDetection tag = getFirstDetection();
                if (tag != null) {
                    packet.put("Vision", "Tag " + tag.id + " found at " + String.format("%.1f", tag.ftcPose.range) + " in");
                    return false;
                }

                packet.put("Vision", "Waiting for tag... " + String.format("%.1f", elapsed) + "s");
                return true;
            }
        };
    }

    /**
     * Rotates the robot to center on a detected tag.
     * Requires you to pass in the Drive so it can call setDrivePowers.
     */
    public Action AutoAimAction(Drive drive, int tagId, double timeoutSeconds) {
        return new Action() {
            private long startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) startTime = System.nanoTime();
                double elapsed = (System.nanoTime() - startTime) / 1e9;

                if (elapsed > timeoutSeconds) {
                    drive.rrDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    packet.put("AutoAim", "Timeout");
                    return false;
                }

                AprilTagDetection tag = getDetectionById(tagId);
                if (tag == null) {
                    packet.put("AutoAim", "Tag " + tagId + " not visible");
                    return true; // Keep trying
                }

                double rotation = getAutoAimRotation(tag.ftcPose.bearing);
                drive.rrDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), rotation));

                packet.put("AutoAim", "Bearing: " + String.format("%.1f", tag.ftcPose.bearing) + " deg");

                // Aligned when rotation output is 0 (within deadzone)
                if (rotation == 0) {
                    drive.rrDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    packet.put("AutoAim", "Aligned!");
                    return false;
                }

                return true;
            }
        };
    }


    public void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            if (Thread.currentThread().isInterrupted()) return;
        }
        ExposureControl ec = visionPortal.getCameraControl(ExposureControl.class);
        if (ec.getMode() != ExposureControl.Mode.Manual) {
            ec.setMode(ExposureControl.Mode.Manual);
        }
        ec.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        visionPortal.getCameraControl(GainControl.class).setGain(gain);
    }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }

    private DetectionResult buildResult(AprilTagDetection tag) {
        if (tag == null || tag.metadata == null) return null;

        // Look up shooter settings for this tag + distance
        for (LookupEntry entry : lookupTable) {
            if (entry.tagId == tag.id && tag.ftcPose.range >= entry.minDist && tag.ftcPose.range < entry.maxDist) {
                return new DetectionResult(tag.id, tag.ftcPose.range, tag.ftcPose.bearing, entry.power, entry.targetRPM);
            }
        }

        // Tag found but no matching lookup entry for this distance
        return new DetectionResult(tag.id, tag.ftcPose.range, tag.ftcPose.bearing, -1, -1);
    }
}