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
    Servo inStop;
    Servo outRotation;
    Servo outSwivel;
    DcMotor intakeMotor;
    Follower follower;

    Pose startPose = new Pose(9, 87, Math.toRadians(90));
    Pose placeSample = new Pose(13, 132, Math.toRadians(135)); // 12,128
    Pose pickupSample1 = new Pose(28, 123, Math.toRadians(0));
    Pose pickupSample2 = new Pose(9, 135, Math.toRadians(0));
    Pose pickupSample3 = new Pose(28, 132, Math.toRadians(45));
//    Pose pickupSample4 = new Pose(28, 121.5, Math.toRadians(180));

    PathChain scorePreloadedSample, pickupSample1Path, scoreSample1, pickupSample2Path, scoreSample2, pickupSample3Path, scoreSample3;//, pickupSample4Path, scoreSample4;

    void slideTo(DcMotor slide, Integer target) {
        slide.setTargetPosition(target);

        if (slide.getCurrentPosition() > target + 10) {
            slide.setPower(-0.8);
        } else if (slide.getCurrentPosition() < target - 10) { // Claw: 0.15 open, 0.26 closed
            slide.setPower(0.8);
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

    void horizontalSlideTo(Integer target) {
        slideTo(horizontalSlide, target);
    }

    void verticalSlideTo(Integer target) {
        slideTo(verticalSlide, target);
    }

    private void placePart1() {
        verticalSlideTo(1800); // Up

        outRotation.setPosition(0.62); // Point towards the basket
    }

    private void placePart2() {
        outClaw.setPosition(0.15); // Open

        sleep(1000);

        outClaw.setPosition(0.26); // Close

        sleep(500);

        outRotation.setPosition(0.45); // Point claw up

        sleep(500);

        verticalSlideTo(500); // Down
    }

    private void place() {
        placePart1();

        sleep(1000);

        placePart2();
    }

    private void pickup() {
        inRotation.setPosition(0.57);

        sleep(400);

        intakeMotor.setPower(1.0);

        sleep(400);

        horizontalSlideTo(400); // Extend to grab sample

        sleep(200);

        inStop.setPosition(0.37);

        horizontalSlideTo(0); // Retract after grabbing it

        sleep(500);

        inRotation.setPosition(0.04);

        sleep(500);

        outRotation.setPosition(0.98); // Point claw down
        outSwivel.setPosition(0.17);
        outClaw.setPosition(0.15); // Open

        sleep(100);

        verticalSlideTo(50); // Down

        outClaw.setPosition(0.26); // Close

        intakeMotor.setPower(0.0);

        sleep(300);

        verticalSlideTo(500); // Kinda up
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

            if (!follower.isBusy()) {
                switch (state) {
                    case 0:
                        placePart1();

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
                        placePart1();

                        follower.followPath(scoreSample2, true);
                        pathTimer.resetTimer();
                        state = 5;

                        break;
                    case 5:
                        placePart2();

                        follower.followPath(pickupSample3Path, true);
                        pathTimer.resetTimer();
                        state = 6;

                        break;
                    case 6:
                        pickup();
                        placePart1();

                        follower.followPath(scoreSample3, true);
                        pathTimer.resetTimer();
                        state = 9;

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
                    case 9:
                        placePart2();
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
