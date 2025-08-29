package org.firstinspires.ftc.teamcode.teleop;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.pedro.FConstants;
import org.firstinspires.ftc.teamcode.pedro.LConstants;
import org.firstinspires.ftc.teamcode.services.Service.PathfindingService;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot
 * on FTC Dashboard (192/168/43/1:8080/dash). You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@TeleOp(group = "Teleop Test", name = "Localization Test With Auto Position")
public class LocalizationTestWithAutoPosition extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private PathChain containerPath;
    private PathfindingService PathfindingService;
    private boolean isTeleop = true;
    private final Pose startPose = new Pose(136,112,-90);
    private PathfindingService.Node endOfPath = new PathfindingService.Node(136,112);

    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PathfindingService = new PathfindingService(follower, poseUpdater);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        poseUpdater = new PoseUpdater(hardwareMap, FConstants.class, LConstants.class);
        poseUpdater.setStartingPose(startPose);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setDirection(leftRearMotorDirection);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(rightFrontMotorDirection);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setDirection(rightRearMotorDirection);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the FTC
     * Dashboard telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        poseUpdater.update();
        dashboardPoseTracker.update();

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // this is strafing
        double rx = gamepad1.right_stick_x;

        // Check if A button was pressed to start auto positioning+
        if (gamepad1.a && isTeleop) {
            isTeleop = false;

            PathfindingService.Node start = new PathfindingService.Node((int) poseUpdater.getPose().getX(), (int) poseUpdater.getPose().getY());
            List<PathfindingService.Node> path = PathfindingService.findPath(start, endOfPath, 1);

            PathBuilder pathBuilder = follower.pathBuilder();

            for (int i = 0; i < path.size() - 1; i++) {
                PathfindingService.Node current = path.get(i);
                PathfindingService.Node next = path.get(i + 1);

                // Create poses with same heading
                Pose currentPose = new Pose(current.x, current.y, -90);
                Pose nextPose = new Pose(next.x, next.y, -90);

                // Add straight line segment
                pathBuilder.addPath(new BezierLine(new Point(currentPose), new Point(nextPose)))
                        .setConstantHeadingInterpolation(-90);
            }
            containerPath = pathBuilder.build();

            follower.followPath(containerPath, true);
        }

        if (gamepad1.xWasPressed()) {
            endOfPath = new PathfindingService.Node((int) poseUpdater.getPose().getX(), (int) poseUpdater.getPose().getY());
        }

        if (isTeleop) {
            // Manual teleop control
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
        } else {
            // Auto mode - update follower
            follower.update();

            // Check if path is finished
            if (!follower.isBusy()) {
                isTeleop = true; // Switch back to teleop when done
            }

            // Optional: Allow manual override by pressing B
            if (gamepad1.b) {
                isTeleop = true;
            }
        }

        telemetryA.addData("x", poseUpdater.getPose().getX());
        telemetryA.addData("y", poseUpdater.getPose().getY());
        telemetryA.addData("heading", poseUpdater.getPose().getHeading());
        telemetryA.addData("total heading", poseUpdater.getTotalHeading());
        telemetryA.addData("pose", poseUpdater.getPose());
        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
