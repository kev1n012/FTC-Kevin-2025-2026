package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name = "Dodge_Blue")
public class Dodge_Blue extends LinearOpMode {

    private DcMotorEx flywheelR, flywheelL, intake, helper_motor;
    private Servo shooter_servo;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int TICKS_PER_REVOLUTION = 28;
    private final double SHOOTER_POWER = 0.75;
    private static int DESIRED_FLYWHEEL_SPEED = 3900;
    private final double INTAKE_SHOOT_POWER = -0.6;
    private final double OPEN_SERVO_POS = 0.25;
    private final double AUTO_AIM_GAIN = 0.015;

    public double cm(double centimeters) { return centimeters / 2.54; }

    private double calculateRPM(DcMotorEx motor) {
        return (motor.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Hardware Initialization
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");
        flywheelL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        helper_motor = hardwareMap.get(DcMotorEx.class, "helper_motor");
        shooter_servo = hardwareMap.get(Servo.class, "shooter_servo");

        flywheelR.setDirection(DcMotor.Direction.FORWARD);
        flywheelL.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // Vision Initialization
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Build the Path
        Action trajectory = drive.actionBuilder(initialPose)
                .lineToX(cm(20))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(trajectory);
        visionPortal.close();
    }
}