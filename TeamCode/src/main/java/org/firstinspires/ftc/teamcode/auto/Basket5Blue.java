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
@Autonomous(name = "Basket Auto")
public class Basket5Blue extends LinearOpMode {
    DcMotor verticalSlide;
    DcMotor horizontalSlide;
    Servo outClaw;
    Servo inRotation;
    Servo inStop;
    Servo outRotation;
    Servo outSwivel;
    DcMotor intakeMotor;
    Follower follower;

    Pose startPose = new Pose(9, 87, Math.toRadians(-90));
    Pose placeSample = new Pose(11.5, 130, Math.toRadians(-48)); // 12,128
    Pose placeSample2 = new Pose(11.5, 130, Math.toRadians(-48));
    Pose pickupSample1 = new Pose(22, 122.5, Math.toRadians(0));
    Pose shimmySample1 = new Pose(22, 123.5, Math.toRadians(12));
    Pose pickupSample2 = new Pose(22, 130.5, Math.toRadians(0));
    Pose shimmySample2 = new Pose(22, 131.5, Math.toRadians(12));

    boolean actionStarted = false;
//    Pose pickupSample4 = new Pose(28, 121.5, Math.toRadians(180));

    PathChain scorePreloadedSample, pickupSample1Path, shimmySample1Path, shimmyBackSample1Path, scoreSample1, pickupSample2Path, shimmySample2Path, shimmyBackSample2Path, scoreSample2, pickupSample3Path, scoreSample3;//, pickupSample4Path, scoreSample4;

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
        verticalSlideTo(1800); // Up

        outRotation.setPosition(0.29); // Point towards the basket
    }

    private void asyncPlacePart1() {
        new Thread(this::placePart1).start();
    }

    private void placePart2() {
        outClaw.setPosition(0.15); // Open

        sleep(350);

        outClaw.setPosition(0.26); // Close
        outRotation.setPosition(0.45); // Point claw up

        sleep(300);

        verticalSlideTo(500); // Down
    }

    private void pickup() {
        inStop.setPosition(0.86);
        inRotation.setPosition(0.89);
        intakeMotor.setPower(1.0);

        sleep(750);


        horizontalSlideTo(600); // Extend to grab sample

        sleep(300);
    }

    private void transfer() {
        horizontalSlideTo(0); // Retract after grabbing it

        sleep(300);

        inRotation.setPosition(0.25); // Rotate back to slide sample to claw :D
        inStop.setPosition(0.37);
        outRotation.setPosition(0.97); // Point claw down
        outSwivel.setPosition(0.85);
        outClaw.setPosition(0.15); // Open

        sleep(200);

        verticalSlideTo(0); // Down

        sleep(400);

        outClaw.setPosition(0.26); // Close
        waitWithFollowerUpdate(200); // Wait while still updating follower
        new Thread(() -> {
            intakeMotor.setPower(0.0);
            inStop.setPosition(0.86);
        }).start();
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
                .addPath(new BezierLine(startPose, placeSample))
                .setLinearHeadingInterpolation(startPose.getHeading(), placeSample.getHeading())
                .build();

        pickupSample1Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample1))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample1.getHeading())
                .build();

        shimmySample1Path = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample1, shimmySample1))
                .setLinearHeadingInterpolation(pickupSample1.getHeading(), shimmySample1.getHeading())
                .build();

        shimmyBackSample1Path = follower.pathBuilder()
                .addPath(new BezierLine(shimmySample1, pickupSample1))
                .setLinearHeadingInterpolation(shimmySample1.getHeading(), pickupSample1.getHeading())
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shimmySample1, placeSample))
                .setLinearHeadingInterpolation(shimmySample1.getHeading(), placeSample.getHeading())
                .build();

        pickupSample2Path = follower.pathBuilder()
                .addPath(new BezierLine(placeSample, pickupSample2))
                .setLinearHeadingInterpolation(placeSample.getHeading(), pickupSample2.getHeading())
                .build();

        shimmySample2Path = follower.pathBuilder()
                .addPath(new BezierLine(pickupSample2, shimmySample2))
                .setLinearHeadingInterpolation(pickupSample2.getHeading(), shimmySample2.getHeading())
                .build();

        shimmyBackSample2Path = follower.pathBuilder()
                .addPath(new BezierLine(shimmySample2, pickupSample2))
                .setLinearHeadingInterpolation(shimmySample2.getHeading(), pickupSample2.getHeading())
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(shimmySample2, placeSample2))
                .setLinearHeadingInterpolation(shimmySample2.getHeading(), placeSample2.getHeading())
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
                        follower.followPath(scorePreloadedSample, 0.75, true);
                        pathTimer.resetTimer();
                        state = 1;

                        break;
                    case 1:
                        placePart2();

                        follower.followPath(pickupSample1Path, true);
                        pathTimer.resetTimer();
                        state = 2;

                        break;
                    case 2:
                        pickup();
                        follower.followPath(shimmySample1Path);
                        pathTimer.resetTimer();
                        state = 22; // New intermediate state
                        break;

                    case 21:
                        follower.followPath(shimmyBackSample1Path);
                        pathTimer.resetTimer();
                        state = 22; // Another intermediate state
                        break;

                    case 22:
                        transfer();
                        placePart1();
                        follower.followPath(scoreSample1, true);
                        pathTimer.resetTimer();
                        state = 3;
                        break;
                    case 3:
                        placePart2();

                        follower.followPath(pickupSample2Path, true);
                        pathTimer.resetTimer();
                        state = 4;

                        break;
                    case 4:
                        pickup();
                        follower.followPath(shimmySample2Path);
                        pathTimer.resetTimer();
                        state = 42; // New intermediate state
                        break;
                    case 41:
                        follower.followPath(shimmyBackSample2Path);
                        pathTimer.resetTimer();
                        state = 42;
                        break;
                    case 42:
                        transfer();
                        placePart1();

                        follower.followPath(scoreSample2, true);
                        pathTimer.resetTimer();
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
                        placePart2();
                        outRotation.setPosition(0.97);
                        sleep(250);
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
