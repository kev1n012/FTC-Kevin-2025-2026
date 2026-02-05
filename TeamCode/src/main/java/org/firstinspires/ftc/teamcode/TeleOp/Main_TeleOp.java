package org.firstinspires.ftc.teamcode.TeleOp;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Main_TeleOp")
public class Main_TeleOp extends OpMode {

    private static final double DEADZONE_DEGREES = 2.0;
    private double rightFlywheelSpeed = 0;
    private double leftFlywheelSpeed = 0;

    private static final double OPEN_SERVO_POS = 0.25;
    private int DESIRED_FLYWHEEL_SPEED = 3900;
    private int CLOSE_SHOOTER_SERVO = 1500;

    private static final double AUTO_AIM_GAIN = 0.01;
    private static final double INTAKE_POWER = -0.3;
    private static final double DESIRED_DISTANCE_INCHES = 78.7;
    private static final double DISTANCE_GAIN = 0.02;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final int TICKS_PER_REVOLUTION = 28;
    private static final double SLOW_MODE_SCALAR = 0.3;

    private static double SHOOTER_POWER = 0.80;

    private DcMotorEx front_left, back_left, front_right, back_right;
    private DcMotorEx flywheelR, flywheelL, intake;
    private DcMotorEx helper_motor;
    private Servo shooter_servo;
    private IMU imu;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        initializeHardware();
        initializeVision();
        configureMotors();
        initializeIMU();
        shooter_servo.setPosition(0);
        telemetry.addData("Status", "Fully Initialized");
    }

    private void initializeHardware() {
        front_left = hardwareMap.get(DcMotorEx.class, "front_left");
        back_left = hardwareMap.get(DcMotorEx.class, "back_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        back_right = hardwareMap.get(DcMotorEx.class, "back_right");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");
        flywheelL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter_servo = hardwareMap.get(Servo.class, "shooter_servo");
        helper_motor = hardwareMap.get(DcMotorEx.class, "helper_motor");
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

        flywheelR.setDirection(DcMotor.Direction.FORWARD);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        helper_motor.setDirection(DcMotor.Direction.FORWARD);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotorRunMode(front_left);
        setMotorRunMode(back_left);
        setMotorRunMode(front_right);
        setMotorRunMode(back_right);
        setMotorRunMode(flywheelR);
        setMotorRunMode(flywheelL);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        helper_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    }

    @Override
    public void loop() {
        double speedMultiplier = (gamepad1.left_bumper) ? SLOW_MODE_SCALAR : 1.0;

        rightFlywheelSpeed = calculateRPM(flywheelR, TICKS_PER_REVOLUTION);
        leftFlywheelSpeed = calculateRPM(flywheelL, TICKS_PER_REVOLUTION);

        if(gamepad1.y){SHOOTER_POWER = 0.75; DESIRED_FLYWHEEL_SPEED = 3900;}
        if(gamepad1.a){SHOOTER_POWER = 0.65 ; DESIRED_FLYWHEEL_SPEED = 3300;}

        updateShooterAndIntake();

        AutoAimResult autoAim = getAutoAimControls();

        driveRobotCentric(
                autoAim.forward * speedMultiplier,
                autoAim.strafe * speedMultiplier,
                autoAim.rotation * speedMultiplier
        );

        if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
            imu.resetYaw();
        }

        updateTelemetry(autoAim.rotation);
        telemetry.update();
    }

    private AutoAimResult getAutoAimControls() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        if (gamepad1.x) {
            AprilTagDetection targetTag = findFirstValidTag();
            if (targetTag != null) {
                rotation = calculateAutoAimRotation(targetTag.ftcPose.bearing);

                double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE_INCHES;
                double robotForward = Range.clip(rangeError * DISTANCE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                forward = forward + robotForward * Math.cos(botHeading);
                strafe = strafe - robotForward * Math.sin(botHeading);
            }
        }

        return new AutoAimResult(rotation, forward, strafe);
    }

    private double calculateAutoAimRotation(double bearingError) {
        if (Math.abs(bearingError) <= DEADZONE_DEGREES) return 0;
        return -bearingError * AUTO_AIM_GAIN;
    }

    public void driveRobotCentric(double forward, double strafe, double rotate) {
        // Robot-oriented drive - use inputs directly without field transformation [1](#4-0)
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Power normalization
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        front_left.setPower(frontLeftPower);
        front_right.setPower(frontRightPower);
        back_left.setPower(backLeftPower);
        back_right.setPower(backRightPower);
    }

    private void updateShooterAndIntake() {
        if (gamepad1.right_bumper) {
            intake.setPower(INTAKE_POWER);
            helper_motor.setPower(0.5);
            flywheelL.setPower(0.0);
            flywheelR.setPower(0.0);
            shooter_servo.setPosition(0);
        } else if (gamepad1.right_trigger >= 0.2f) {
            intake.setPower(INTAKE_POWER);
            helper_motor.setPower(0.5);
            flywheelL.setPower(SHOOTER_POWER);
            flywheelR.setPower(SHOOTER_POWER);
            if (leftFlywheelSpeed > DESIRED_FLYWHEEL_SPEED && rightFlywheelSpeed > DESIRED_FLYWHEEL_SPEED) {
                shooter_servo.setPosition(OPEN_SERVO_POS);
            }
        } else if (gamepad1.left_trigger >= 0.5f) {
            intake.setPower(-INTAKE_POWER);
            helper_motor.setPower(-0.5);
            flywheelL.setPower(-SHOOTER_POWER);
            flywheelR.setPower(-SHOOTER_POWER);
            shooter_servo.setPosition(0.25);
        } else {
            intake.setPower(0.0);
            helper_motor.setPower(0.0);
            flywheelL.setPower(0.0);
            flywheelR.setPower(0.0);
            if (leftFlywheelSpeed > CLOSE_SHOOTER_SERVO && rightFlywheelSpeed > CLOSE_SHOOTER_SERVO) {
                shooter_servo.setPosition(0);
            }
        }
    }

    private void updateTelemetry(double rotate) {
        updateAutoAimTelemetry();
        updateAprilTagTelemetry();

        telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Shooter Servo Pos", shooter_servo.getPosition());
        telemetry.addData("Intake Power", helper_motor.getPower());
        telemetry.addData("SHOOTER_POWER", SHOOTER_POWER);

        telemetry.addData("Shooter RPM", "R:%.1f L:%.1f", rightFlywheelSpeed, leftFlywheelSpeed);
        telemetry.addData("Flywheel Goal", DESIRED_FLYWHEEL_SPEED);
        telemetry.addData("Drive RPM", "FL:%.1f FR:%.1f BL:%.1f BR:%.1f",
                calculateRPM(front_left, TICKS_PER_REVOLUTION),
                calculateRPM(front_right, TICKS_PER_REVOLUTION),
                calculateRPM(back_left, TICKS_PER_REVOLUTION),
                calculateRPM(back_right, TICKS_PER_REVOLUTION));
    }

    private void updateAutoAimTelemetry() {
        if (gamepad1.x) {
            AprilTagDetection targetTag = findFirstValidTag();
            if (targetTag != null) {
                telemetry.addData("Auto-Aim", "ID %d, Bearing: %.1fÂ°", targetTag.id, targetTag.ftcPose.bearing);
            }
        }
    }

    private void updateAprilTagTelemetry() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# Tags Detected", detections.size());
        for (AprilTagDetection d : detections) {
            if (d.metadata != null) {
                telemetry.addData("Tag ID", d.id);
                telemetry.addData("Range/Bearing", "%.1f in / %.1f deg", d.ftcPose.range, d.ftcPose.bearing);
            }
        }
    }

    private AprilTagDetection findFirstValidTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) return detection;
        }
        return null;
    }

    private double calculateRPM(DcMotorEx motor, double ticksPerRevolution) {
        return (motor.getVelocity() / ticksPerRevolution) * 60.0;
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            if (Thread.currentThread().isInterrupted()) return;
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        visionPortal.getCameraControl(GainControl.class).setGain(gain);
    }

    @Override
    public void stop() { visionPortal.close(); }

    private static class AutoAimResult {
        final double rotation;
        final double forward;
        final double strafe;
        AutoAimResult(double rotation, double forward, double strafe) {
            this.rotation = rotation;
            this.forward = forward;
            this.strafe = strafe;
        }
    }
}