//TOP OF THE FLOCK
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
@Autonomous(name = "Bar Auto")
public class BarAuto extends LinearOpMode {
    DcMotor verticalSlide;
    DcMotor horizontalSlide;
    Servo outClaw;
    Servo inRotation;
    Servo inStop;
    Servo outRotation;
    Servo outSwivel;
    DcMotor intakeMotor;
    Follower follower;

    Pose startPose = new Pose(0, 0, Math.toRadians(180));
    Pose placeSpec = new Pose(36, 7, Math.toRadians(180)); // 12,128
    Pose stupidMiddlePoint = new Pose(20, 7, Math.toRadians(180));
    Pose parkCords = new Pose(3, -42, Math.toRadians(180)); //LOL!!!!!

    PathChain scorePreloadedSample, scoreToMiddle, park;

    boolean actionStarted = false;
//    Pose pickupSample4 = new Pose(28, 121.5, Math.toRadians(180));

    void slideTo(DcMotor slide, Integer target) {
        slide.setTargetPosition(target);

        if (slide.getCurrentPosition() > target + 10) {
            slide.setPower(-1.0);
        } else if (slide.getCurrentPosition() < target - 10) { // Claw: 0.15 open, 0.26 closed
            slide.setPower(1.0);
        } else {
            return;
        }

        while (Math.abs(target - slide.getCurrentPosition()) > 10) {
            follower.update();
            sleep(10);

            telemetry.addData("Target", target);
            telemetry.addData("Current", slide.getCurrentPosition());
            telemetry.update();
        }
    }

    private void waitWithFollowerUpdate(long millis) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < millis) {
            follower.update();
            telemetry.update();
            sleep(10);
        }
    }

    void horizontalSlideTo(Integer target) {
        slideTo(horizontalSlide, target);
    }
    void verticalSlideTo(Integer target) {
        slideTo(verticalSlide, target);
    }

    private void placePart1() {
        outRotation.setPosition(0.2); // Point towards the basket
        verticalSlideTo(500); // Up
        outRotation.setPosition(0.04);
        verticalSlideTo(1300);
    }

    private void hangPart1() {
        verticalSlideTo(500);
        outClaw.setPosition(0.15);
    }

    private void asyncPlacePart1() {
        new Thread(this::placePart1).start();
    }

    @Override
    public void runOpMode() {
        verticalSlide = hardwareMap.dcMotor.get("verticalSlide");
        horizontalSlide = hardwareMap.dcMotor.get("horizontalSlide");
        outClaw = hardwareMap.servo.get("OutClaw");
        inRotation = hardwareMap.servo.get("InRotation");
        inStop = hardwareMap.servo.get("InStop");
        outRotation = hardwareMap.servo.get("OutRotation");
        outSwivel = hardwareMap.servo.get("OutSwivel");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        outClaw.setPosition(0.26); // Close

        verticalSlide.setTargetPosition(0);
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalSlide.setTargetPosition(0);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Timer pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);



        scorePreloadedSample = follower.pathBuilder()
                .addPath(new BezierLine(startPose, placeSpec))
                .setLinearHeadingInterpolation(startPose.getHeading(), placeSpec.getHeading())
                .build();

        scoreToMiddle = follower.pathBuilder()
                .addPath(new BezierLine(placeSpec, stupidMiddlePoint))
                .setLinearHeadingInterpolation(placeSpec.getHeading(), stupidMiddlePoint.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(stupidMiddlePoint, parkCords))
                .setLinearHeadingInterpolation(stupidMiddlePoint.getHeading(), parkCords.getHeading())
                .build();

//        pickupSample4Path = follower.pathBuilder()
//                .addPath(new BezierLine(placeSample, pickupSample4))
//                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample4.getHeading())
//                .build();
//
//        scoreSample4 = follower.pathBuilder()
//                .addPath(new BezierLine(pickupSample4, placeSample))
//                .setLinearHeadingInterpolation(pickupSample4.getHeading(), placeSample.getHeading())
//                .build();

        int state = 0;
        int currentPathState = 0; // This is going to be so disgusting later, but I don't care

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("State: ", state);
            telemetry.addData("isbusy", follower.isBusy());

            if (!follower.isBusy()) { //this sucks
                switch (state) {
                    case 0:
                        asyncPlacePart1();
                        follower.followPath(scorePreloadedSample, 0.6, true);
                        pathTimer.resetTimer();
                        state = 1;
                        break;
                    case 1:
                        hangPart1();
                        follower.followPath(scoreToMiddle);
                        follower.followPath(park);
                        state = 5;
                        break;
//                    case 7:
//                        place();
//
//                        follower.followPath(pickupSample4Path, true);
//                        pathTimer.resetTimer();
//                        state = 8;
//
//                        break;
//                    case 8:
//                        pickup();
//
//                        follower.followPath(scoreSample4, true);
//                        pathTimer.resetTimer();
//                        state = 9;
//
//                        break;
                    case 5:
                        outRotation.setPosition(0.97);
                        sleep(1000);
                        verticalSlideTo(0);
                        state = 6;
                        break;
                    case 6:
                        pathTimer.resetTimer();

                        state = -1;

                    default:
                        // KTHXBYE
                        requestOpModeStop();
                }
            } else {
                sleep(100);
            }

            telemetry.update();
        }
    }
}
