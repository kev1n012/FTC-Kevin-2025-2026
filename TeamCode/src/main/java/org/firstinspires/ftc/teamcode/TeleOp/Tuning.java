package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;



@TeleOp(name = "Tuning")
public class Tuning extends OpMode {

    // Robot classes

    // Drive motors — still declared here because TeleOp drive is manual mecanum, not RoadRunner
    private DcMotorEx front_left, back_left, front_right, back_right;
    private IMU imu;

    // Drive config
    private static final double SLOW_MODE_SCALAR = 0.3;
    private static final int    TICKS_PER_REVOLUTION = 28;

    // Auto-aim config (kept here because the distance-correction math uses the IMU, which lives here)
    private static final double DESIRED_DISTANCE_INCHES = 78.7;
    private static final double DISTANCE_GAIN           = 0.02;
    private static final double MAX_AUTO_SPEED          = 0.5;


    double SHOOTER_POWER = 1.0;
    int RPM = 3000;


    @Override
    public void init() {
        // ── drive motors ──────────────────────────────────────────────
        front_left  = hardwareMap.get(DcMotorEx.class, "front_left");
        back_left   = hardwareMap.get(DcMotorEx.class, "back_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        back_right  = hardwareMap.get(DcMotorEx.class, "back_right");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ── IMU ───────────────────────────────────────────────────────
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();

        telemetry.addData("Status", "Fully Initialized");
    }

    @Override
    public void loop() {
        double speedMultiplier = gamepad1.left_bumper ? SLOW_MODE_SCALAR : 1.0;

        if(gamepad1.dpad_up){
            SHOOTER_POWER = SHOOTER_POWER + 0.05;
            shooter.setPower(SHOOTER_POWER);
        } else if(gamepad1.dpad_down){
            SHOOTER_POWER = SHOOTER_POWER - 0.05;
            shooter.setPower(SHOOTER_POWER);
        }

        if(gamepad1.dpad_right){
            RPM = RPM + 100;
            shooter.setTargetRPM(RPM);
        } else if(gamepad1.dpad_left){
            RPM = RPM - 100;
            shooter.setTargetRPM(RPM);
        }

        // ── shooter preset buttons (same as original Y / A) ──────────
        if (gamepad1.y) {
            shooter.setPower(0.75);
            shooter.setTargetRPM(3900);
        }
        if (gamepad1.a) {
            shooter.setPower(0.65);
            shooter.setTargetRPM(3300);
        }

        // ── shooter + intake state machine (mirrors original exactly) ─
        if (gamepad1.right_bumper) {
            // intake only, flywheels off
            intake.collect();
            shooter.stop();

        } else if (gamepad1.right_trigger >= 0.2f) {
            // intake + shoot: flywheels ramp up, servo opens at speed
            intake.collect();
            shooter.shoot();   // returns true/false but we don't need it in TeleOp;
            // it keeps itself running as long as we keep calling it

        } else if (gamepad1.left_trigger >= 0.5f) {
            // full reverse / eject
            intake.eject();
            shooter.stop();

        } else {
            // idle — everything off
            intake.stop();
            shooter.stop();
        }

        // ── drive input ───────────────────────────────────────────────
        double forward  = -gamepad1.left_stick_y;
        double strafe   =  gamepad1.left_stick_x;
        double rotation =  gamepad1.right_stick_x;


        driveRobotCentric(
                forward  * speedMultiplier,
                strafe   * speedMultiplier,
                rotation * speedMultiplier
        );

        // ── IMU reset (both sticks pressed) ───────────────────────────
        if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
            imu.resetYaw();
        }

        // ── telemetry ─────────────────────────────────────────────────
        updateTelemetry(vision.getDetectionResult());
        telemetry.update();
    }

    // ===================================================================
    // DRIVE  (mecanum math — same as original)
    // ===================================================================

    private void driveRobotCentric(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        front_left.setPower(fl);
        front_right.setPower(fr);
        back_left.setPower(bl);
        back_right.setPower(br);
    }

    // ===================================================================
    // TELEMETRY
    // ===================================================================

    private void updateTelemetry(Vision.DetectionResult target) {
        // shooter
        telemetry.addData("Shooter RPM",    "R:%.1f  L:%.1f",  shooter.getRightRPM(), shooter.getLeftRPM());
        telemetry.addData("Shooter Target", "Power: %.2f  RPM: %d", shooter.getPower(), shooter.getTargetRPM());
        telemetry.addData("At Speed",       shooter.isAtSpeed());

        // intake
        telemetry.addData("Intake",         "Motor: %.2f  Helper: %.2f", intake.getIntakePower(), intake.getHelperPower());

        // vision — every visible tag
        telemetry.addData("Tags Detected",  vision.getAllDetections().size());
        for (AprilTagDetection d : vision.getAllDetections()) {
            if (d.metadata != null) {
                telemetry.addData("Tag " + d.id, "%.1f cm / %.1f deg", d.ftcPose.range * 2.54, d.ftcPose.bearing);
            }
        }

        // vision — active auto-aim target + lookup result
        if (gamepad1.x && target != null) {
            telemetry.addData("Auto-Aim Target", "ID %d  Range: %.1f in  Bearing: %.1f deg",
                    target.tagId, target.range, target.bearing);
            if (target.hasSettings) {
                telemetry.addData("Lookup Table", "Power: %.2f  RPM: %d", target.power, target.targetRPM);
            } else {
                telemetry.addData("Lookup Table", "No entry for this distance!");
            }
        }

        // drive
        telemetry.addData("Heading",   "%.1f deg", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Drive RPM", "FL:%.0f  FR:%.0f  BL:%.0f  BR:%.0f",
                rpm(front_left), rpm(front_right), rpm(back_left), rpm(back_right));
    }

    private double rpm(DcMotorEx motor) {
        return (motor.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }

    // ===================================================================
    // CLEANUP
    // ===================================================================

    @Override
    public void stop() {
        vision.close();
    }
}