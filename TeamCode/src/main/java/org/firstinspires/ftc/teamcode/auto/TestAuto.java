package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.FConstants;
import org.firstinspires.ftc.teamcode.pedro.LConstants;

@Autonomous(name = "Pedro Pathing Testing Auto")
public class TestAuto extends LinearOpMode {
    Pose startPose = new Pose(132, 32, 90);
    Pose controlPose1 = new Pose(139.88036410923277,132.01560468140443,0);
    Pose controlPose2 = new Pose(104.86345903771131,119.65669700910273,0);
    Pose goalPose = new Pose(50,120,180);

    @Override
    public void runOpMode() {
        Timer pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        PathChain goalPath = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, controlPose1, controlPose2, goalPose))
                .setTangentHeadingInterpolation()
                .build();

        int pathState = 0;

        Telemetry telemetry1 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drawing.drawRobot(startPose, "#4CAF50");
        Drawing.sendPacket();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            switch (pathState) {
                case 0:
                    follower.followPath(goalPath, true);
                    pathTimer.resetTimer();
                    pathState = -1;
                    break;
                default:
                    break;
            }

            Drawing.drawRobot(follower.getPose(), "#4CAF50");
            Drawing.sendPacket();

            telemetry1.addData("x", follower.getPose().getX());
            telemetry1.addData("y", follower.getPose().getY());
            telemetry1.addData("heading", follower.getPose().getHeading());
            telemetry1.update();
        }
    }
}
