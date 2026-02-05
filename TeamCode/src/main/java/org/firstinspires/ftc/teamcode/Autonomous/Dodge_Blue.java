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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Autonomous(name = "Dodge_Blue")
public class Dodge_Blue extends LinearOpMode {

    private DcMotorEx flywheelR, flywheelL, intake, helper_motor;
    private Servo shooter_servo;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int TICKS_PER_REVOLUTION = 28;
    private final double SHOOTER_POWER = 0.6;
    private static int DESIRED_FLYWHEEL_SPEED = 3300;
    private static int CLOSE_SERVO_SPEED = 3000;
    private final double INTAKE_SHOOT_POWER = -0.6;
    private final double OPEN_SERVO_POS = 0.25;
    private final double CLOSE_SERVO_POS = 0.0;
    private final double AUTO_AIM_GAIN = 0.015;

    // Shooting function
    public boolean SERVO_STATE = false;
    public int TOTAL_BALLS = 3;
    public  int CURRENT_BALLS = 0;
    public int RPM_BALL_COUNTER = 3000;

    public boolean HAS_COUNTED = false;

    public double cm(double centimeters) { return centimeters / 2.54; }

    private double calculateRPM(DcMotorEx motor) {
        return (motor.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }

    public Action Shoot() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double rightRPM = calculateRPM(flywheelR);
                double leftRPM = calculateRPM(flywheelL);

                flywheelL.setPower(SHOOTER_POWER);
                flywheelR.setPower(SHOOTER_POWER);
                intake.setPower(INTAKE_SHOOT_POWER);
                helper_motor.setPower(1.0);


                telemetry.addData("Current balls", CURRENT_BALLS);

                telemetry.update();
                if (rightRPM > DESIRED_FLYWHEEL_SPEED && leftRPM > DESIRED_FLYWHEEL_SPEED) {
                    shooter_servo.setPosition(OPEN_SERVO_POS);
                    SERVO_STATE = true;

                    if(CURRENT_BALLS == TOTAL_BALLS){
                        return false;
                    }

                    if(rightRPM < RPM_BALL_COUNTER || leftRPM < RPM_BALL_COUNTER){{
                        if(HAS_COUNTED){
                            CURRENT_BALLS++;
                        }
                        if(rightRPM > RPM_BALL_COUNTER || leftRPM > RPM_BALL_COUNTER){
                            return true;
                        }
                        HAS_COUNTED = false;

                        if(CURRENT_BALLS == TOTAL_BALLS){
                            shooter_servo.setPosition(CLOSE_SERVO_POS);
                            return false;

                        }
                    }
                    }
                }
                return true;
            }
        };
    }



    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryPacket packet = new TelemetryPacket();



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
                .strafeToLinearHeading(new Vector2d(-43, 0), Math.toRadians(0))
                .stopAndAdd(Shoot())
                .build();

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(trajectory);
        visionPortal.close();
    }
}