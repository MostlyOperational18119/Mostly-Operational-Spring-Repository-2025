package org.firstinspires.ftc.teamcode.services.Service.Planner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FTC2024FieldConfig implements FieldConfig{
    @Override
    public FieldNode[] getFixedFieldNodes() {
        return new FieldNode[] {
                // Submersible
                new FieldNode(
                        FieldNode.NodeType.OBSTACLE,
                        new Pose2D(DistanceUnit.METER, 1.8288, 1.8288, AngleUnit.DEGREES, 0),
                        0.6096,
                        1.2192
                ),

                // Left Wall
                new FieldNode(
                        FieldNode.NodeType.OBSTACLE,
                        new Pose2D(DistanceUnit.METER, -0.25, 1.8288, AngleUnit.DEGREES, 0),
                        0.25,
                        3.6576
                ),
                // Right Wall
                new FieldNode(
                        FieldNode.NodeType.OBSTACLE,
                        new Pose2D(DistanceUnit.METER, 3.9076, 1.8288, AngleUnit.DEGREES, 0),
                        0.25,
                        3.6576
                ),
                // Top Wall
                new FieldNode(
                        FieldNode.NodeType.OBSTACLE,
                        new Pose2D(DistanceUnit.METER, 1.8288, 3.6576, AngleUnit.DEGREES, 0),
                        3.6576,
                        0.25
                ),
                // Bottom Wall
                new FieldNode(
                        FieldNode.NodeType.OBSTACLE,
                        new Pose2D(DistanceUnit.METER, 1.8288, -0.25, AngleUnit.DEGREES, 0),
                        3.6576,
                        0.25
                ),


        };
    }

    @Override
    public FieldNode[] getStartingDynamicFieldNodes() {
        return new FieldNode[] {

        };
    }
}
