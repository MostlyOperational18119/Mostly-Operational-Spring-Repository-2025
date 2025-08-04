package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.PINPOINT;

        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 14.515; // In kg

        FollowerConstants.leftFrontMotorName = "motorFL";
        FollowerConstants.leftRearMotorName = "motorBL";
        FollowerConstants.rightFrontMotorName = "motorFR";
        FollowerConstants.rightRearMotorName = "motorBR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        // Tuning D:
        FollowerConstants.xMovement = 67.8989;
        FollowerConstants.yMovement = 47.0148;

        FollowerConstants.forwardZeroPowerAcceleration = -33.49;
        FollowerConstants.lateralZeroPowerAcceleration = -74.9233;

        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.012, 0.0, 0.0001, 0.6, 0.0);
    }
}
