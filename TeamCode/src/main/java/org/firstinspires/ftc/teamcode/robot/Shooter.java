package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//  THIS IS TESTING AND WILL NOT BE USED IN THE COMPETITION


public class Shooter {

    // Hardware
    private final DcMotorEx flywheelR, flywheelL;
    private final Servo shooter_servo;

    // Config
    private static final double OPEN_SERVO_POS = 0.25;
    private static final int TICKS_PER_REVOLUTION = 28;
    private static final double SERVO_OPEN_DURATION = 5.0;

    // Power curve config
    private static final double RAMP_DURATION = 1.0;
    private static final double MIN_POWER = 0.2;

    // Tunable shooter settings (can be changed at runtime via lookup table)
    private double shooterPower = 0.80;
    private int desiredFlywheelSpeed = 3900;
    private static final double TARGET_PERCENT = 0.97;

    // Internal state
    private long rampStartTime = 0;
    private long servoOpenTime = 0;
    private boolean servoOpened = false;

    public Shooter(HardwareMap hardwareMap) {
        shooter_servo = hardwareMap.get(Servo.class, "shooter_servo");
        flywheelL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");

        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelR.setDirection(DcMotor.Direction.FORWARD);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);
        flywheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ===================================================================
    // SETTINGS - called before shooting to apply lookup table values
    // ===================================================================

    public void setPower(double power) {
        this.shooterPower = power;
    }

    public void setTargetRPM(int rpm) {
        this.desiredFlywheelSpeed = rpm;
    }

    // ===================================================================
    // CORE LOGIC - plain methods, usable in TeleOp loop directly
    // ===================================================================

    /**
     * Call every loop iteration to pre-spin flywheels at half power.
     * Safe to call repeatedly.
     */
    public void spinUp() {
        shooter_servo.setPosition(0);
        flywheelL.setPower(0.5 * shooterPower);
        flywheelR.setPower(0.5 * shooterPower);
    }

    /**
     * Call every loop iteration while shooting.
     * Handles ramping, RPM check, servo timing, and timeout internally.
     * Returns true while still shooting, false when complete.
     */
    public boolean shoot() {
        if (rampStartTime == 0) {
            rampStartTime = System.nanoTime();
            servoOpened = false;
            servoOpenTime = 0;
        }

        double elapsed = (System.nanoTime() - rampStartTime) / 1e9;

        // Timeout safety - eject if flywheels never reach speed
        if (elapsed > 5.0) {
            shooter_servo.setPosition(OPEN_SERVO_POS);
            flywheelL.setPower(-shooterPower);
            flywheelR.setPower(-shooterPower);
            resetShootState();
            return false;
        }

        // Ramp power up smoothly
        double rampedPower = calculateRampedPower(elapsed);
        flywheelL.setPower(rampedPower);
        flywheelR.setPower(rampedPower);

        double rightRPM = getRightRPM();
        double leftRPM = getLeftRPM();
        boolean atSpeed = (leftRPM * TARGET_PERCENT) > desiredFlywheelSpeed
                && (rightRPM * TARGET_PERCENT) > desiredFlywheelSpeed;

        // Open servo once we hit target speed
        if (atSpeed && !servoOpened) {
            shooter_servo.setPosition(OPEN_SERVO_POS);
            servoOpenTime = System.nanoTime();
            servoOpened = true;
        } else if (!atSpeed) {
            shooter_servo.setPosition(0);
        }

        // Once servo has been open long enough, close and finish
        if (servoOpened) {
            double servoElapsed = (System.nanoTime() - servoOpenTime) / 1e9;
            if (servoElapsed >= SERVO_OPEN_DURATION) {
                shooter_servo.setPosition(0);
                resetShootState();
                return false; // Done shooting
            }
        }

        return true; // Still shooting
    }

    /**
     * Call every loop iteration to stop everything.
     * Safe to call repeatedly.
     */
    public void stop() {
        flywheelR.setPower(0);
        flywheelL.setPower(0);
        shooter_servo.setPosition(0);
        resetShootState();
    }

    // ===================================================================
    // GETTERS - read state from TeleOp for telemetry or conditionals
    // ===================================================================

    public double getRightRPM() {
        return (flywheelR.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }

    public double getLeftRPM() {
        return (flywheelL.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }

    public boolean isAtSpeed() {
        return (getLeftRPM() * TARGET_PERCENT) > desiredFlywheelSpeed
                && (getRightRPM() * TARGET_PERCENT) > desiredFlywheelSpeed;
    }

    public int getTargetRPM() {
        return desiredFlywheelSpeed;
    }

    public double getPower() {
        return shooterPower;
    }

    // ===================================================================
    // ACTIONS - thin wrappers for autonomous use with RoadRunner
    // ===================================================================

    public Action SpinUpAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spinUp();
                return false; // One-shot, sets power and exits
            }
        };
    }

    public Action ShootAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return shoot(); // Keeps running until shoot() returns false
            }
        };
    }

    public Action StopAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stop();
                return false;
            }
        };
    }

    // ===================================================================
    // PRIVATE HELPERS
    // ===================================================================

    private void resetShootState() {
        rampStartTime = 0;
        servoOpenTime = 0;
        servoOpened = false;
    }

    private double calculateRampedPower(double elapsedTime) {
        double t = Math.min(elapsedTime / RAMP_DURATION, 1.0);
        // Ease-in-out S-curve
        double curve = t < 0.5
                ? 4 * t * t * t
                : 1 - Math.pow(-2 * t + 2, 3) / 2;
        return MIN_POWER + (shooterPower - MIN_POWER) * curve;
    }
}