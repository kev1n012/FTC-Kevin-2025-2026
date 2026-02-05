package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Vision;

/**
 * COMPREHENSIVE AUTONOMOUS EXAMPLE
 *
 * This autonomous demonstrates:
 * ✅ Drive wrapper (cm to inches conversion)
 * ✅ Vision-based auto-aim
 * ✅ Lookup table shooter settings
 * ✅ Shooter with smooth power ramping
 * ✅ Intake timed collection
 * ✅ Proper sequencing with Actions
 *
 * The pattern: drive → aim → apply lookup settings → shoot → collect → repeat
 */
@Autonomous(name = "EXAMPLE - Full Featured Auto")
public class Test_autonomous extends LinearOpMode {

    // Robot components
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;

    // Field positions (in centimeters for readability)
    private static final double START_X = 0;
    private static final double START_Y = 0;

    private static final double SHOOT_POSITION_1_X = -100;  // First shooting spot
    private static final double SHOOT_POSITION_1_Y = 0;

    private static final double COLLECT_X = 150;           // Collection zone
    private static final double COLLECT_Y = 50;

    private static final double SHOOT_POSITION_2_X = 100;  // Second shooting spot
    private static final double SHOOT_POSITION_2_Y = 20;

    private static final double PARK_X = 200;              // Parking spot
    private static final double PARK_Y = 0;

    // AprilTag configuration
    private static final int TARGET_TAG_ID = 20;  // Change based on alliance color

    @Override
    public void runOpMode() {
        // ══════════════════════════════════════════════════════════════
        // INITIALIZATION
        // ══════════════════════════════════════════════════════════════

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize all robot components
        drive   = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        vision  = new Vision(hardwareMap);

        // Set starting pose (if using odometry/localization)
        // drive.rrDrive.setPose(new Pose2d(START_X * 0.393701, START_Y * 0.393701, 0));

        telemetry.addData("Status", "✓ Ready - Press START");
        telemetry.addData("Target Tag", TARGET_TAG_ID);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ══════════════════════════════════════════════════════════════
        // AUTONOMOUS SEQUENCE
        // ══════════════════════════════════════════════════════════════

        try {
            // ──────────────────────────────────────────────────────────
            // SEQUENCE 1: Drive to first shooting position and shoot
            // ──────────────────────────────────────────────────────────

            telemetry.addData("Phase", "1. Driving to shoot position");
            telemetry.update();

            Actions.runBlocking(drive.driveTo(SHOOT_POSITION_1_X, SHOOT_POSITION_1_Y, 0));

            telemetry.addData("Phase", "1. Aiming and shooting");
            telemetry.update();

            // Shoot with vision-guided settings
            shootWithVision(TARGET_TAG_ID, 5.0);

            // ──────────────────────────────────────────────────────────
            // SEQUENCE 2: Drive to collection zone and collect
            // ──────────────────────────────────────────────────────────

            telemetry.addData("Phase", "2. Driving to collection zone");
            telemetry.update();

            Actions.runBlocking(drive.driveTo(COLLECT_X, COLLECT_Y, 45));

            telemetry.addData("Phase", "2. Collecting game elements");
            telemetry.update();

            Actions.runBlocking(intake.CollectForAction(2.5));  // Collect for 2.5 seconds

            // ──────────────────────────────────────────────────────────
            // SEQUENCE 3: Return to shooting position and shoot again
            // ──────────────────────────────────────────────────────────

            telemetry.addData("Phase", "3. Returning to shoot");
            telemetry.update();

            Actions.runBlocking(drive.driveTo(SHOOT_POSITION_2_X, SHOOT_POSITION_2_Y, 0));

            telemetry.addData("Phase", "3. Aiming and shooting");
            telemetry.update();

            shootWithVision(TARGET_TAG_ID, 5.0);

            // ──────────────────────────────────────────────────────────
            // SEQUENCE 4: Park
            // ──────────────────────────────────────────────────────────

            telemetry.addData("Phase", "4. Parking");
            telemetry.update();

            Actions.runBlocking(drive.driveTo(PARK_X, PARK_Y, 0));

            telemetry.addData("Status", "✓ Autonomous Complete!");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.update();
        } finally {
            // Always cleanup
            vision.close();
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // REUSABLE ACTION: SHOOT WITH VISION
    // ═══════════════════════════════════════════════════════════════════

    /**
     * Complete shooting sequence using vision:
     * 1. Wait for tag detection
     * 2. Auto-aim at target
     * 3. Apply lookup table settings
     * 4. Shoot
     *
     * This is the recommended pattern - combines everything into one method.
     */
    private void shootWithVision(int tagId, double timeout) {
        // Wait up to 2 seconds to detect the tag
        Actions.runBlocking(vision.WaitForDetectionAction(2.0));

        // Auto-aim at the target (rotates robot to center on tag)
        Actions.runBlocking(vision.AutoAimAction(drive, tagId, 3.0));

        // Get the detection result with lookup table settings
        Vision.DetectionResult target = vision.getDetectionResultById(tagId);

        if (target != null && target.hasSettings) {
            // Apply optimal power/RPM from lookup table
            shooter.setPower(target.power);
            shooter.setTargetRPM(target.targetRPM);

            telemetry.addData("Shooter Settings", "Power: %.2f  RPM: %d", target.power, target.targetRPM);
            telemetry.addData("Distance", "%.1f inches", target.range);
            telemetry.update();
        } else if (target != null) {
            // Tag visible but no lookup entry - use defaults
            telemetry.addData("Warning", "No lookup table entry for %.1f in", target.range);
            telemetry.update();
            // shooter already has default power/RPM from initialization
        } else {
            telemetry.addData("Warning", "Tag not found - using defaults");
            telemetry.update();
        }

        // Shoot sequence
        Actions.runBlocking(shooter.SpinUpAction());     // Pre-spin at half power
        Actions.runBlocking(intake.CollectAction());     // Start intake
        Actions.runBlocking(shooter.ShootAction());      // Ramp, wait for speed, open servo, wait, close
        Actions.runBlocking(shooter.StopAction());       // Stop shooter
        Actions.runBlocking(intake.StopAction());        // Stop intake
    }

    // ═══════════════════════════════════════════════════════════════════
    // ALTERNATIVE: MANUAL SHOOTING (no vision)
    // ═══════════════════════════════════════════════════════════════════

    /**
     * Shoot without vision - just use preset power/RPM.
     * Useful if vision isn't available or for testing.
     */
    private void shootManual(double power, int targetRPM) {
        shooter.setPower(power);
        shooter.setTargetRPM(targetRPM);

        Actions.runBlocking(shooter.SpinUpAction());
        Actions.runBlocking(intake.CollectAction());
        Actions.runBlocking(shooter.ShootAction());
        Actions.runBlocking(shooter.StopAction());
        Actions.runBlocking(intake.StopAction());
    }

    // ═══════════════════════════════════════════════════════════════════
    // ALTERNATIVE: CUSTOM SHOOTING ACTION (inline, like your old code)
    // ═══════════════════════════════════════════════════════════════════

    /**
     * If you need more control than the built-in Actions provide,
     * you can still create custom inline Actions.
     *
     * This example shows manual control over the shooting sequence
     * similar to your old autonomous code.
     */
    private Action customShootSequence(int tagId) {
        return new Action() {
            private long startTime = 0;
            // Use instance variables instead of static to avoid Java 8 limitation
            private final int STATE_AIMING = 0;
            private final int STATE_SHOOTING = 1;
            private final int STATE_DONE = 2;
            private int currentState = STATE_AIMING;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) startTime = System.nanoTime();
                double elapsed = (System.nanoTime() - startTime) / 1e9;

                // Overall timeout
                if (elapsed > 10.0) {
                    shooter.stop();
                    intake.stop();
                    return false;
                }

                switch (currentState) {
                    case STATE_AIMING:
                        // Aim for 3 seconds
                        Vision.DetectionResult target = vision.getDetectionResultById(tagId);

                        if (target != null) {
                            double rotation = vision.getAutoAimRotation(target.bearing);
                            drive.rrDrive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                                    new com.acmerobotics.roadrunner.Vector2d(0, 0), rotation));

                            // Apply lookup settings
                            if (target.hasSettings) {
                                shooter.setPower(target.power);
                                shooter.setTargetRPM(target.targetRPM);
                            }
                        }

                        if (elapsed > 3.0) {
                            drive.rrDrive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                                    new com.acmerobotics.roadrunner.Vector2d(0, 0), 0));
                            currentState = STATE_SHOOTING;
                        }
                        break;

                    case STATE_SHOOTING:
                        // Shoot until complete
                        intake.collect();
                        boolean stillShooting = shooter.shoot();

                        if (!stillShooting) {
                            shooter.stop();
                            intake.stop();
                            currentState = STATE_DONE;
                        }
                        break;

                    case STATE_DONE:
                        return false;
                }

                return true;
            }
        };
    }

    // ═══════════════════════════════════════════════════════════════════
    // EXAMPLE: COMPLEX TRAJECTORY WITH EMBEDDED ACTIONS
    // ═══════════════════════════════════════════════════════════════════

    /**
     * Shows how to embed shooting directly in a trajectory builder
     * (similar to your old autonomous style)
     */
    private void exampleComplexTrajectory() {
        Action complexPath = drive.rrDrive.actionBuilder(drive.rrDrive.localizer.getPose())
                .lineToX(100 * 0.393701)  // Drive forward
                .turn(Math.toRadians(30)) // Turn
                .stopAndAdd(new Action() {
                    // Inline shooting action
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        // Get settings from vision
                        Vision.DetectionResult target = vision.getDetectionResult();
                        if (target != null && target.hasSettings) {
                            shooter.setPower(target.power);
                            shooter.setTargetRPM(target.targetRPM);
                        }

                        // Shoot
                        intake.collect();
                        boolean stillShooting = shooter.shoot();

                        if (!stillShooting) {
                            shooter.stop();
                            intake.stop();
                            return false;
                        }
                        return true;
                    }
                })
                .lineToY(50 * 0.393701)   // Continue path
                .build();

        Actions.runBlocking(complexPath);
    }
}