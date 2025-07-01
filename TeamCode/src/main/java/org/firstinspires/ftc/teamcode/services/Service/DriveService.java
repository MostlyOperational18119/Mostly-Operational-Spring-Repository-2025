package org.firstinspires.ftc.teamcode.services.Service;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;

import java.util.Arrays;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

public class DriveService implements Runnable {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor verticalSlide;
    DcMotor horizontalSlide;
    HardwareMap hardwareMap;
    GoBildaPinpointDriver pinpoint;

    final private LinkedBlockingQueue<DriveServiceInput> inputQueue;

    private Boolean isOnlyWheels;

    // Constants for FTCLib Kinematics, double check at some point
    // 23 cm wide 30 cm long
    final Translation2d motorFLLocation = new Translation2d(-0.115, 0.15);
    final Translation2d motorFRLocation = new Translation2d(0.115, 0.15);
    final Translation2d motorBLLocation = new Translation2d(-0.115, -0.15);
    final Translation2d motorBRLocation = new Translation2d(0.115, -0.15);

    final double MAX_SPEED = 10.0; // m/sec

    MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            motorFLLocation,
            motorFRLocation,
            motorBLLocation,
            motorBRLocation
    );

    public DriveService(HardwareMap hardwareMap, LinkedBlockingQueue<DriveServiceInput> inputQueue, Boolean isOnlyWheels) {
        this.hardwareMap = hardwareMap;
        this.inputQueue = inputQueue;
        this.isOnlyWheels = isOnlyWheels;

        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

//        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        if (!isOnlyWheels) {
            horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
            verticalSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
        }
    }

    @Override
    public void run() {
        try {
            while (true) {
                DriveServiceInput input = inputQueue.poll(5, TimeUnit.SECONDS);

                // It's not impossible
                if (input != null ) {
                    if (input.mode == DriveServiceInput.DriveServiceInputMode.MANUAL && !input.doBreak) {
                        motorFL.setPower(input.flSpeed);
                        motorFR.setPower(input.frSpeed);
                        motorBL.setPower(input.blSpeed);
                        motorBR.setPower(input.brSpeed);
                        if (!isOnlyWheels) {
                            verticalSlide.setPower(input.verticalSlideSpeed);
                            horizontalSlide.setPower(input.horizontalSlideSpeed);
                        }
                    } else if (input.mode == DriveServiceInput.DriveServiceInputMode.MANUAL && input.doBreak) {
                        motorFL.setPower(-motorFL.getPower());
                        motorFR.setPower(-motorFR.getPower());
                        motorBL.setPower(-motorBL.getPower());
                        motorBR.setPower(-motorBR.getPower());
                        if (!isOnlyWheels) {
                            verticalSlide.setPower(0.0);
                            horizontalSlide.setPower(0.0);
                        }

                        Thread.sleep(50);

                        motorFL.setPower(0.0);
                        motorFR.setPower(0.0);
                        motorBL.setPower(0.0);
                        motorBR.setPower(0.0);
                    } else if (input.mode == DriveServiceInput.DriveServiceInputMode.PLAN) {
                        Log.i("DriveService", "Control: " + Arrays.toString(input.control));
                        double posControlX = input.control[0];
                        double posControlY = input.control[1];
                        double rotControl = input.control[2];

                        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                                posControlX,
                                posControlY,
                                rotControl
                        );

                        MecanumDriveWheelSpeeds wheelspeeds = kinematics.toWheelSpeeds(chassisSpeeds);


                        motorFL.setPower(wheelspeeds.frontLeftMetersPerSecond * 20);
                        motorFR.setPower(wheelspeeds.frontRightMetersPerSecond * 20);
                        motorBL.setPower(wheelspeeds.rearLeftMetersPerSecond * 20);
                        motorBR.setPower(wheelspeeds.rearRightMetersPerSecond * 20);

                        Log.i(
                                "DriveService",
                                String.format(
                                        "FL(%.2f) FR(%.2f) BL(%.2f) BR(%.2f)",
                                        wheelspeeds.frontLeftMetersPerSecond*20,
                                        wheelspeeds.frontRightMetersPerSecond*20,
                                        wheelspeeds.rearLeftMetersPerSecond*20,
                                        wheelspeeds.rearRightMetersPerSecond*20
                                )
                        );
                    }
                }
            }
        } catch (InterruptedException e) {
            // If we were interrupted, we should (probably) be exiting
        }
    }

    public void setup() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        verticalSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
