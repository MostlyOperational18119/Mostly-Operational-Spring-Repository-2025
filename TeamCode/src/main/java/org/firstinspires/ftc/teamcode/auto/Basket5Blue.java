package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.FConstants;
import org.firstinspires.ftc.teamcode.pedro.LConstants;

@Autonomous(name = "Basket 5 Blue Auto")
public class Basket5Blue extends LinearOpMode {
    Pose startPose = new Pose(35.5, 10, Math.toRadians(90));
    Pose placeSample = new Pose(13, 133.5, Math.toRadians(135));
    Pose pickupSample1 = new Pose(25.7, 23, Math.toRadians(180));
    Pose pickupSample2 = new Pose(25.7, 12, Math.toRadians(180));
    Pose pickupSample3 = new Pose(33, 14.8, Math.toRadians(135));
    Pose pickupSample4 = new Pose(28, 121.5, Math.toRadians(180));

    PathChain scorePreloadedSample, pickupSample1Path, scoreSample1, pickupSample2Path, scoreSample2, pickupSample3Path, scoreSample3, pickupSample4Path, scoreSample4;

    @Override
    public void runOpMode() {
        Timer pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);


        scorePreloadedSample = follower.pathBuilder()
                .addPath(new BezierLine(startPose, placeSample))
                .setLinearHeadingInterpolation(startPose.getHeading(), placeSample.getHeading())
                .build();

        pickupSample1Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample1))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample1.getHeading())
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample1, placeSample))
                .setLinearHeadingInterpolation(pickupSample1.getHeading(), placeSample.getHeading())
                .build();

        pickupSample2Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample2))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample2.getHeading())
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample2, placeSample))
                .setLinearHeadingInterpolation(pickupSample2.getHeading(), placeSample.getHeading())
                .build();

        pickupSample3Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample3))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample3.getHeading())
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample3, placeSample))
                .setLinearHeadingInterpolation(pickupSample3.getHeading(), placeSample.getHeading())
                .build();

        pickupSample4Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample4))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample4.getHeading())
                .build();

        scoreSample4 = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample4, placeSample))
                .setLinearHeadingInterpolation(pickupSample4.getHeading(), placeSample.getHeading())
                .build();

        int state = 0;
        int currentPathState = 0; // This is going to be so disgusting later, but I don't care

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            if (!follower.isBusy()) {
                switch (state) {
                    case 0:
                        follower.followPath(scorePreloadedSample, true);
                        pathTimer.resetTimer();
                        state = 1;

                        break;
                    case 1:
                        follower.followPath(pickupSample1Path, true);
                        pathTimer.resetTimer();
                        state = 2;

                        break;
                    case 2:
                        follower.followPath(scoreSample1, true);
                        pathTimer.resetTimer();
                        state = 3;

                        break;
                    case 3:
                        follower.followPath(pickupSample2Path, true);
                        pathTimer.resetTimer();
                        state = 4;

                        break;
                    case 4:
                        follower.followPath(scoreSample2, true);
                        pathTimer.resetTimer();
                        state = 5;

                        break;
                    case 5:
                        follower.followPath(pickupSample3Path, true);
                        pathTimer.resetTimer();
                        state = 6;

                        break;
                    case 6:
                        follower.followPath(scoreSample3, true);
                        pathTimer.resetTimer();
                        state = 7;

                        break;
                    case 7:
                        follower.followPath(pickupSample4Path, true);
                        pathTimer.resetTimer();
                        state = 8;

                        break;
                    case 8:
                        follower.followPath(scoreSample4, true);
                        pathTimer.resetTimer();
                        state = -1;

                        break;

                    default:
                        // KTHXBYE
                        requestOpModeStop();
                }
            }
        }
    }
}
