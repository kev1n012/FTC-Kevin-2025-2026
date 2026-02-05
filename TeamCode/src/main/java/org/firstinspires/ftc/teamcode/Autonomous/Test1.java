package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Vision;

/**
 * Custom Autonomous Sequence:
 * 1. Drive BACK 110cm
 * 2. Power up shooter based on Vision lookup table
 * 3. Shoot
 * 4. Stop shooter
 * 5. Turn 25 degrees LEFT (counterclockwise)
 * 6. Strafe LEFT 20cm
 * 7. Drive FORWARD
 * 8. Run intake for 5 seconds
 */
@Autonomous(name = "Custom Sequence - Back Shoot Left Collect")
public class Test1 extends LinearOpMode {

    // Robot components
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;

    // Configuration
    private static final int TARGET_TAG_ID = 1;  // Change based on your target (1 = Blue, 2 = Red, etc.)

    @Override
    public void runOpMode() {
        // ══════════════════════════════════════════════════════════════
        // INITIALIZATION
        // ══════════════════════════════════════════════════════════════

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        drive   = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        vision  = new Vision(hardwareMap);

        telemetry.addData("Status", "✓ Ready - Press START");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ══════════════════════════════════════════════════════════════
        // AUTONOMOUS SEQUENCE
        // ══════════════════════════════════════════════════════════════

        try {
            // ──────────────────────────────────────────────────────────
            // STEP 1: Drive BACK 110cm
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Step", "1. Driving back 110cm");
            telemetry.update();

            // Negative X = backward
            Actions.runBlocking(drive.driveTo(-110, 0, 0));

            // ──────────────────────────────────────────────────────────
            // STEP 2: Power up shooter based on Vision lookup table
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Step", "2. Detecting target and setting power");
            telemetry.update();

            // Wait for AprilTag detection (up to 2 seconds)
            Actions.runBlocking(vision.WaitForDetectionAction(2.0));

            // Get optimal shooter settings from lookup table
            Vision.DetectionResult target = vision.getDetectionResultById(TARGET_TAG_ID);

            if (target != null && target.hasSettings) {
                // Lookup table found settings for this distance
                shooter.setPower(target.power);
                shooter.setTargetRPM(target.targetRPM);

                telemetry.addData("Vision", "✓ Tag detected");
                telemetry.addData("Distance", "%.1f inches", target.range);
                telemetry.addData("Shooter Power", "%.2f", target.power);
                telemetry.addData("Shooter RPM", target.targetRPM);
            } else {
                // Fallback to default settings if no lookup entry or tag not found
                shooter.setPower(0.75);
                shooter.setTargetRPM(3900);

                telemetry.addData("Vision", "⚠ Using default settings");
                telemetry.addData("Shooter Power", "0.75 (default)");
                telemetry.addData("Shooter RPM", "3900 (default)");
            }
            telemetry.update();

            // ──────────────────────────────────────────────────────────
            // STEP 3: Shoot
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Step", "3. Shooting");
            telemetry.update();

            Actions.runBlocking(shooter.SpinUpAction());   // Pre-spin at half power
            Actions.runBlocking(intake.CollectAction());   // Start feeding into shooter
            Actions.runBlocking(shooter.ShootAction());    // Ramp up, shoot, wait

            telemetry.addData("Step", "3. ✓ Shot complete");
            telemetry.update();

            // ──────────────────────────────────────────────────────────
            // STEP 4: Stop shooter
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Step", "4. Stopping shooter");
            telemetry.update();

            Actions.runBlocking(shooter.StopAction());
            Actions.runBlocking(intake.StopAction());

            // ──────────────────────────────────────────────────────────
            // STEP 5: Turn 25 degrees LEFT (counterclockwise)
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Step", "5. Turning 25° left");
            telemetry.update();

            // Positive angle = counterclockwise (left)
            Actions.runBlocking(drive.TurnTo(25));

            // ──────────────────────────────────────────────────────────
            // STEP 6: Strafe LEFT 20cm
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Step", "6. Strafing left 20cm");
            telemetry.update();

            // Positive Y = left
            // Current position is x=-110, y=0, heading=25
            // New position will be x=-110, y=20, heading=25
            Actions.runBlocking(drive.driveTo(-110, 20, 25));

            // ──────────────────────────────────────────────────────────
            // STEP 7: Drive FORWARD
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Step", "7. Driving forward");
            telemetry.update();

            // Move forward from current position
            // Let's go forward 50cm from where we are
            // Current: x=-110, new: x=-60
            Actions.runBlocking(drive.driveTo(-60, 20, 25));

            // ──────────────────────────────────────────────────────────
            // STEP 8: Run intake for 5 seconds
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Step", "8. Collecting for 5 seconds");
            telemetry.update();

            Actions.runBlocking(intake.CollectForAction(5.0));

            telemetry.addData("Step", "8. ✓ Collection complete");
            telemetry.update();

            // ──────────────────────────────────────────────────────────
            // SEQUENCE COMPLETE
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Status", "✓✓✓ Autonomous Complete! ✓✓✓");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.update();

        } finally {
            // Always cleanup
            vision.close();
        }
    }
}