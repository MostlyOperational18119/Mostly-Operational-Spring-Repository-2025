package org.firstinspires.ftc.teamcode.alex;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.FConstants;
import org.firstinspires.ftc.teamcode.pedro.LConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTagAuto")

public class AprilTagAuto extends LinearOpMode {

    Pose startPosition = new Pose(0, 0, Math.toRadians(0));
    Pose leftPosition = new Pose(12, 0, Math.toRadians(0));
    Pose rightPosition = new Pose(-12, 0, Math.toRadians(0));
    Pose backPosition = new Pose(0, -12, Math.toRadians(0));
    private VisionPortal visionPortal;
    private AprilTagProcessor apriltag;
    private int AprilTagId  = -1;
    Follower follower;
    PathChain left, right, back;
    @Override

    public void runOpMode() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        left = follower.pathBuilder()
            .addPath(new BezierLine(startPosition, leftPosition))
            .setLinearHeadingInterpolation(startPosition.getHeading(), leftPosition.getHeading())
            .build();

        right = follower.pathBuilder()
                .addPath(new BezierLine(startPosition, rightPosition))
                .setLinearHeadingInterpolation(startPosition.getHeading(), rightPosition.getHeading())
                .build();

        back = follower.pathBuilder()
                .addPath(new BezierLine(startPosition, backPosition))
                .setLinearHeadingInterpolation(startPosition.getHeading(), backPosition.getHeading())
                .build();

        apriltag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(apriltag);
        visionPortal = builder.build();

        while (AprilTagId == -1 && !isStarted() && !isStopRequested()) {
            List<AprilTagDetection> currentDetections = apriltag.getDetections();
            for (AprilTagDetection detection : currentDetections ) {
                AprilTagId = detection.id;
                telemetry.addData("ID: ", detection.id);
            }

            if (AprilTagId == -1) {
                telemetry.addData("AprilTag", "No tags detected");
            }

            telemetry.update();
            sleep(20);
        }

        waitForStart();
        while (opModeIsActive()) {
            follower.update();

            if (!follower.isBusy()) {
                switch (AprilTagId) {
                    case 21:
                        follower.followPath(left);
                    case 22:
                        follower.followPath(right);
                    case 23:
                        follower.followPath(back);
                }
            }
        }


    }
}
