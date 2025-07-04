package org.firstinspires.ftc.teamcode.services.Service.Planner;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FieldNode {
    public enum NodeType {
        OBSTACLE,
        GOAL,
        ROBOT,
        SCORING_ELEMENT,
        UNKNOWN
    }

    public enum GoalType {
        PUSHBOT,
        BASKET,
        SCALING_ELEMENT,
        CLIMB,
        UNKNOWN
    }

    public enum NodeShape {
        RECTANGLE,
        TRIANGLE,
        CIRCLE,
        UNKNOWN
    }

    public NodeType type;

    // Location
    public Pose2D position;
    public double width;
    public double height;
    public NodeShape shape;
    public Alliance alliance;

    public FieldNode(NodeType type, Pose2D position, double width, double height) {
        this.type = type;
        this.position = position;
        this.width = width;
        this.height = height;
        this.shape = NodeShape.RECTANGLE;
        this.alliance = Alliance.NONE;
    }

    public FieldNode(NodeType type, Pose2D position, double width, double height, Alliance alliance) {
        this.type = type;
        this.position = position;
        this.width = width;
        this.height = height;
        this.shape = NodeShape.RECTANGLE;
        this.alliance = alliance;
    }

    public FieldNode(NodeType type, Pose2D position, double width, double height, NodeShape shape) {
        this.type = type;
        this.position = position;
        this.width = width;
        this.height = height;
        this.shape = shape;
        this.alliance = Alliance.NONE;
    }

    public FieldNode(NodeType type, Pose2D position, double width, double height, NodeShape shape, Alliance alliance) {
        this.type = type;
        this.position = position;
        this.width = width;
        this.height = height;
        this.shape = shape;
        this.alliance = alliance;
    }
}
