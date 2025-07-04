package org.firstinspires.ftc.teamcode.services.Communication;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class VisionServiceOutput {
    public Pose2D pose2DFromRobot;
    public Pose3D pose3DFromRobot;

    public enum VisionResultType {
        ROBOT,
        SCORING_ELEMENT
    }

    public VisionResultType type;
    
    public VisionServiceOutput(Pose2D pose2DFromRobot, Pose3D pose3D, VisionResultType type) {
        this.pose2DFromRobot = pose2DFromRobot;
        this.pose3DFromRobot = pose3D;
        this.type = type;
    }
}
