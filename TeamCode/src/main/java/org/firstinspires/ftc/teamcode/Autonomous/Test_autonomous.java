package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "Basic Auto")
public class Test_autonomous extends LinearOpMode {

    //The lib uses inches now we can see in cm
    public double cm(double centimeters) {
        return centimeters / 2.54;
    }


    @Override
    public void runOpMode() {
        // Initialize the drive with the starting position
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Define your path
        Action trajectory = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(cm(70), cm(50)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Execute the path
        Actions.runBlocking(trajectory);
    }
}
