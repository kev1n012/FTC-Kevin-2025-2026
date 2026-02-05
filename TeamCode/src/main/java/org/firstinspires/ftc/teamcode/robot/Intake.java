package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//  THIS IS TESTING AND WILL NOT BE USED IN THE COMPETITION



public class Intake {

    // Hardware
    private final DcMotorEx intake;
    private final DcMotorEx helper_motor;

    // Config
    private static final double INTAKE_POWER = 0.3;
    private static final double HELPER_POWER = 0.5;

    // Internal state for timed actions
    private long timedStartTime = 0;
    private double timedDuration = 0;
    private boolean timedEject = false;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        helper_motor = hardwareMap.get(DcMotorEx.class, "helper_motor");

        intake.setDirection(DcMotor.Direction.REVERSE);
        helper_motor.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        helper_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        helper_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ===================================================================
    // CORE LOGIC - plain methods, usable in TeleOp loop directly
    // ===================================================================

    /** Start collecting. Safe to call every loop iteration. */
    public void collect() {
        intake.setPower(-INTAKE_POWER);
        helper_motor.setPower(HELPER_POWER);
    }

    /** Start ejecting. Safe to call every loop iteration. */
    public void eject() {
        intake.setPower(INTAKE_POWER);
        helper_motor.setPower(-HELPER_POWER);
    }

    /** Stop intake. Safe to call every loop iteration. */
    public void stop() {
        intake.setPower(0);
        helper_motor.setPower(0);
        resetTimedState();
    }

    /**
     * Timed collect/eject - call every loop iteration.
     * Returns true while still running, false when the time is up.
     * @param seconds How long to run
     * @param eject   true = eject, false = collect
     */
    public boolean runFor(double seconds, boolean eject) {
        if (timedStartTime == 0) {
            timedStartTime = System.nanoTime();
            timedDuration = seconds;
            timedEject = eject;
        }

        // Apply the correct direction
        if (timedEject) {
            eject();
        } else {
            collect();
        }

        double elapsed = (System.nanoTime() - timedStartTime) / 1e9;
        if (elapsed >= timedDuration) {
            stop();
            return false; // Done
        }

        return true; // Still running
    }

    // ===================================================================
    // GETTERS
    // ===================================================================

    public double getIntakePower() {
        return intake.getPower();
    }

    public double getHelperPower() {
        return helper_motor.getPower();
    }

    // ===================================================================
    // ACTIONS - thin wrappers for autonomous use with RoadRunner
    // ===================================================================

    public Action CollectAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                collect();
                return false; // One-shot, sets power and exits
            }
        };
    }

    public Action EjectAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                eject();
                return false;
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

    /** Runs collect or eject for a set duration then stops automatically */
    public Action CollectForAction(double seconds) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return runFor(seconds, false);
            }
        };
    }

    public Action EjectForAction(double seconds) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return runFor(seconds, true);
            }
        };
    }

    // ===================================================================
    // PRIVATE HELPERS
    // ===================================================================

    private void resetTimedState() {
        timedStartTime = 0;
        timedDuration = 0;
        timedEject = false;
    }
}