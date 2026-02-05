package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.MecanumDrive;

//  THIS IS TESTING AND WILL NOT BE USED IN THE COMPETITION


public class Drive {
    public final MecanumDrive rrDrive;
    private static final double CM_TO_INCH = 0.393701;

    public Drive(HardwareMap hardwareMap) {
        this.rrDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public Action driveTo(double xCm, double yCm, double headingDeg) {
        double xInch = xCm * CM_TO_INCH;
        double yInch = yCm * CM_TO_INCH;

        return rrDrive.actionBuilder(rrDrive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(xInch, yInch), Math.toRadians(headingDeg))
                .build();
    }

    public Action TurnTo(double headingDeg) {
        return rrDrive.actionBuilder(rrDrive.localizer.getPose())
                .turnTo(Math.toRadians(headingDeg))
                .build();
    }
    
}




























































































































































































