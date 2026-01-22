package org.firstinspires.ftc.teamcode.TeleOp;

import java.util.List;
import java.lang.Thread;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Main_TeleOp_Fixed")
public class Main_TeleOp extends OpMode {

    private static final double DEADZONE_DEGREES = 2.0;
    private static final double AUTO_AIM_GAIN = 0.01;
    private static final double INTAKE_POWER = 0.3;
    private static final double DESIRED_DISTANCE_INCHES = 78.7;
    private static final double DISTANCE_GAIN = 0.02;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final int TICKS_PER_REVOLUTION = 28;
    private static final double SLOW_MODE_SCALAR = 0.3;

    private static double SHOOTER_POWER = 0;

    private DcMotorEx front_left, back_left, front_right, back_right;
    private DcMotorEx shooterR, shooterL, intake;
    private CRServo shooter_servo;
    private DcMotorEx intake_motor;
    private IMU imu;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        initializeHardware();
        initializeVision();
        configureMotors();
        initializeIMU();
        telemetry.addData("Status", "Fully Initialized with IMU");
    }

    private void initializeHardware() {
        front_left = hardwareMap.get(DcMotorEx.class, "front_left");
        back_left = hardwareMap.get(DcMotorEx.class, "back_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        back_right = hardwareMap.get(DcMotorEx.class, "back_right");

        shooterR = hardwareMap.get(DcMotorEx.class, "flywheelR");
        shooterL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter_servo = hardwareMap.get(CRServo.class, "shooter_servo");
        intake_motor = hardwareMap.get(DcMotorEx.class, "helper_motor");

        imu = hardwareMap.get(IMU.class, "imu");
    }

    private void initializeVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        setManualExposure(6, 250);
    }

    private void configureMotors() {
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.FORWARD);
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter_servo.setDirection(CRServo.Direction.FORWARD);
        intake_motor.setDirection(DcMotor.Direction.FORWARD);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorRunMode(front_left);
        setMotorRunMode(back_left);
        setMotorRunMode(front_right);
        setMotorRunMode(back_right);
        setMotorRunMode(shooterR);
        setMotorRunMode(shooterL);
        setMotorRunMode(intake);
        setMotorRunMode(intake_motor);
    }

    private void setMotorRunMode(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeIMU() {
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void loop() {

        // Update drivetrain with field-oriented control
        double speedMultiplier = 1.0;
        if (gamepad1.left_trigger > 0.1) {
            speedMultiplier = SLOW_MODE_SCALAR;
        }

        if (gamepad1.dpad_up) {
        SHOOTER_POWER = SHOOTER_POWER + 0.05;
        }

        if (gamepad1.dpad_down) {
            SHOOTER_POWER = SHOOTER_POWER - 0.05;
        }

        // Get auto-aim rotation
        AutoAimResult autoAim = getAutoAimControls();

        // Pass the multiplier into the drive function
        driveFieldCentric(
                autoAim.forward * speedMultiplier,
                gamepad1.left_stick_x * speedMultiplier,
                autoAim.rotation * speedMultiplier
        );

        // Combined shooter and intake control
        updateShooterAndIntake();

        if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
            imu.resetYaw();
        }
        // Update telemetry
        updateTelemetry(autoAim.rotation);
        telemetry.update();
    }

    private AutoAimResult getAutoAimControls() {
        if (!gamepad1.x) {
            return new AutoAimResult(gamepad1.right_stick_x, -gamepad1.left_stick_y);
        }

        AprilTagDetection targetTag = findFirstValidTag();
        if (targetTag == null) {
            return new AutoAimResult(gamepad1.right_stick_x, -gamepad1.left_stick_y);
        }

        // Get current robot heading
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Convert camera-relative bearing to field-relative rotation
        double cameraBearing = targetTag.ftcPose.bearing;
        double fieldRelativeRotation = cameraBearing + Math.toDegrees(robotHeading);

        // Calculate rotation for aiming
        double rotation = calculateAutoAimRotation(fieldRelativeRotation);

        // Calculate forward/backward movement for distance control
        double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE_INCHES;
        double forward = Range.clip(rangeError * DISTANCE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

        return new AutoAimResult(rotation, forward);
    }

    private double calculateAutoAimRotation(double bearingError) {
        if (Math.abs(bearingError) <= DEADZONE_DEGREES) {
            return 0;
        }
        return -bearingError * AUTO_AIM_GAIN;  // Negated to fix direction
    }

    private static class AutoAimResult {
        final double rotation;
        final double forward;

        AutoAimResult(double rotation, double forward) {
            this.rotation = rotation;
            this.forward = forward;
        }
    }

    private AprilTagDetection findFirstValidTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                return detection;
            }
        }
        return null;
    }

    private void updateShooterAndIntake() {
        boolean r1Pressed = gamepad1.right_bumper;
        boolean l1Pressed = gamepad1.left_bumper;
        boolean r2Pressed = gamepad1.right_trigger >= 0.2f;

        if (r1Pressed) {
            intake.setPower(INTAKE_POWER);
            intake_motor.setPower(0.5);
            shooter_servo.setPower(0.0);
            shooterL.setPower(0.0);
            shooterR.setPower(0.0);
        } else if (l1Pressed) {
            intake.setPower(0.0);
            intake_motor.setPower(0.5);
            shooter_servo.setPower(1.0);
            shooterL.setPower(SHOOTER_POWER);
            shooterR.setPower(SHOOTER_POWER);
        } else if (r2Pressed){
            intake.setPower(INTAKE_POWER);
            intake_motor.setPower(0.5);
            shooter_servo.setPower(1.0);
            shooterL.setPower(SHOOTER_POWER);
            shooterR.setPower(SHOOTER_POWER);
        } else if (gamepad1.left_trigger >= 0.5f) { // This is so the motor can be reversed to get balls out that are stuck //TODO Fix this
            intake.setPower(-0.3);
            intake_motor.setPower(-0.5);
            shooter_servo.setPower(-1.0);
            shooterL.setPower(-SHOOTER_POWER);
            shooterR.setPower(-SHOOTER_POWER);
        } else {
            intake.setPower(0.0);
            intake_motor.setPower(0.0);
            shooter_servo.setPower(0.0);
            shooterL.setPower(0.0);
            shooterR.setPower(0.0);
        }
    }

    private void updateTelemetry(double rotate) {
        updateAutoAimTelemetry();
        updateAprilTagTelemetry();
        updateSystemTelemetry();
        updateRPMTelemetry();
    }

    private void updateAutoAimTelemetry() {
        if (gamepad1.x){
            AprilTagDetection targetTag = findFirstValidTag();
            if (targetTag != null) {
                telemetry.addData("Auto-Aim", "Target ID %d, Bearing: %.1f°, Deadzone: %.1f°",
                        targetTag.id, targetTag.ftcPose.bearing, DEADZONE_DEGREES);
            }
        }
    }

    private void updateAprilTagTelemetry() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Range", "%.1f inches", detection.ftcPose.range);
                telemetry.addData("Yaw", "%.1f degrees", detection.ftcPose.yaw);
                telemetry.addData("Bearing", "%.1f degrees", detection.ftcPose.bearing);
            }
        }
    }

    private void updateSystemTelemetry() {
        telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Shooter Servo Power", shooter_servo.getPower());
        telemetry.addData("Intake Motor Power", intake_motor.getPower());  // Updated telemetry name
        telemetry.addData("SHOOTER_POWER", SHOOTER_POWER);
    }

    private void updateRPMTelemetry() {
        telemetry.addData("Shooter RPM", "%.1f, %.1f",
                calculateRPM(shooterR, TICKS_PER_REVOLUTION),
                calculateRPM(shooterL, TICKS_PER_REVOLUTION));
        telemetry.addData("Drive RPM", "%.1f, %.1f, %.1f, %.1f",
                calculateRPM(front_left, TICKS_PER_REVOLUTION),
                calculateRPM(front_right, TICKS_PER_REVOLUTION),
                calculateRPM(back_left, TICKS_PER_REVOLUTION),
                calculateRPM(back_right, TICKS_PER_REVOLUTION));
        telemetry.addData("Intake RPM", "%.1f", calculateRPM(intake, TICKS_PER_REVOLUTION));
        telemetry.addData("Intake Motor RPM", "%.1f", calculateRPM(intake_motor, TICKS_PER_REVOLUTION));  // Added RPM for intake motor
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

    private double calculateRPM(DcMotorEx motor, double ticksPerRevolution) {
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60.0;
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        waitForCameraStreaming();
        setCameraControls(exposureMS, gain);
    }

    private void waitForCameraStreaming() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            return;
        }

        telemetry.addData("Camera", "Waiting");
        telemetry.update();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

        telemetry.addData("Camera", "Ready");
        telemetry.update();
    }

    private void setCameraControls(int exposureMS, int gain) {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleepWithInterruptHandling(50);
        }

        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        sleepWithInterruptHandling(20);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleepWithInterruptHandling(20);
    }

    private void sleepWithInterruptHandling(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void driveFieldCentric(double forward, double strafe, double rotate) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Normalize the heading to prevent discontinuities
        botHeading = AngleUnit.normalizeRadians(botHeading);
        telemetry.addData("Heading", botHeading);

        // Rotate the movement vectors by the heading of the robot
        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1.0);

        front_left.setPower((rotY + rotX + rotate) / denominator);
        back_left.setPower((rotY - rotX + rotate) / denominator);
        front_right.setPower((rotY - rotX - rotate) / denominator);
        back_right.setPower((rotY + rotX - rotate) / denominator);
    }
}