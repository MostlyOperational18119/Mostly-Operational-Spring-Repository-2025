package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.FConstants;
import org.firstinspires.ftc.teamcode.pedro.LConstants;

@SuppressWarnings("unused")
@Autonomous(name = "Basket 5 Blue Auto")
public class Basket5Blue extends LinearOpMode {
    DcMotor verticalSlide;
    DcMotor horizontalSlide;
    Servo outClaw;
    Servo inRotation;
    DcMotor intakeMotor;

    Pose startPose = new Pose(35.5, 10, Math.toRadians(90));
    Pose placeSample = new Pose(13, 133.5, Math.toRadians(135));
    Pose pickupSample1 = new Pose(25.7, 23, Math.toRadians(180));
    Pose pickupSample2 = new Pose(25.7, 12, Math.toRadians(180));
    Pose pickupSample3 = new Pose(33, 14.8, Math.toRadians(135));
    Pose pickupSample4 = new Pose(28, 121.5, Math.toRadians(180));

    PathChain scorePreloadedSample, pickupSample1Path, scoreSample1, pickupSample2Path, scoreSample2, pickupSample3Path, scoreSample3, pickupSample4Path, scoreSample4;

    private void place() {
        verticalSlide.setTargetPosition(1000);

        verticalSlide.setPower(0.7);

        while (Math.abs(1000 - verticalSlide.getCurrentPosition()) > 10) {
            sleep(10);
        }

        outClaw.setPosition(0.4);

        sleep(100);

        verticalSlide.setTargetPosition(0);

        verticalSlide.setPower(-0.7);

        while (Math.abs(1000 - verticalSlide.getCurrentPosition()) > 10) {
            sleep(10);
        }
    }

    private void pickup() {
        inRotation.setPosition(0.57);
        intakeMotor.setPower(1.0);

        horizontalSlide.setTargetPosition(1000);

        horizontalSlide.setPower(0.7);

        while (Math.abs(1000 - horizontalSlide.getCurrentPosition()) > 10) {
            sleep(10);
        }

        sleep(100);


    }

    @Override
    public void runOpMode() {
        verticalSlide = hardwareMap.dcMotor.get("verticalSlide");
        horizontalSlide = hardwareMap.dcMotor.get("horizontalSlide");
        outClaw = hardwareMap.servo.get("OutClaw");
        inRotation = hardwareMap.servo.get("InRotation");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");


        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Timer pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);



        scorePreloadedSample = follower.pathBuilder()
                .addPath(new BezierLine(startPose, placeSample))
                .setLinearHeadingInterpolation(startPose.getHeading(), placeSample.getHeading())
                .addTemporalCallback(2, this::place)
                .build();

        pickupSample1Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample1))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample1.getHeading())
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample1, placeSample))
                .setLinearHeadingInterpolation(pickupSample1.getHeading(), placeSample.getHeading())
                .addTemporalCallback(2, this::place)
                .build();

        pickupSample2Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample2))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample2.getHeading())
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample2, placeSample))
                .setLinearHeadingInterpolation(pickupSample2.getHeading(), placeSample.getHeading())
                .addTemporalCallback(2, this::place)
                .build();

        pickupSample3Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample3))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample3.getHeading())
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample3, placeSample))
                .setLinearHeadingInterpolation(pickupSample3.getHeading(), placeSample.getHeading())
                .addTemporalCallback(2, this::place)
                .build();

        pickupSample4Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample4))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample4.getHeading())
                .build();

        scoreSample4 = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample4, placeSample))
                .setLinearHeadingInterpolation(pickupSample4.getHeading(), placeSample.getHeading())
                .addTemporalCallback(2, this::place)
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
                        state = -1;

                        break;
                    case 1:
                        place();

                        follower.followPath(pickupSample1Path, true);
                        pathTimer.resetTimer();
                        state = 2;

                        break;
                    case 2:
                        pickup();

                        follower.followPath(scoreSample1, true);
                        pathTimer.resetTimer();
                        state = 3;

                        break;
                    case 3:
                        place();

                        follower.followPath(pickupSample2Path, true);
                        pathTimer.resetTimer();
                        state = 4;

                        break;
                    case 4:
                        pickup();

                        follower.followPath(scoreSample2, true);
                        pathTimer.resetTimer();
                        state = 5;

                        break;
                    case 5:
                        place();

                        follower.followPath(pickupSample3Path, true);
                        pathTimer.resetTimer();
                        state = 6;

                        break;
                    case 6:
                        pickup();

                        follower.followPath(scoreSample3, true);
                        pathTimer.resetTimer();
                        state = 7;

                        break;
                    case 7:
                        place();

                        follower.followPath(pickupSample4Path, true);
                        pathTimer.resetTimer();
                        state = 8;

                        break;
                    case 8:
                        pickup();

                        follower.followPath(scoreSample4, true);
                        pathTimer.resetTimer();
                        state = 9;

                        break;
                    case 9:
                        place();

                        state = -1;

                    default:
                        // KTHXBYE
                        requestOpModeStop();
                }
            } else {

            }
        }
    }
}
