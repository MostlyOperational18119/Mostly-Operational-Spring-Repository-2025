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
        FollowerConstants.mass = 5.987419; // In kg

        FollowerConstants.leftFrontMotorName = "motorFL";
        FollowerConstants.leftRearMotorName = "motorBL";
        FollowerConstants.rightFrontMotorName = "motorFR";
        FollowerConstants.rightRearMotorName = "motorBR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        // Tuning D:
        FollowerConstants.xMovement = 62.321;
        FollowerConstants.yMovement = 52.6343;

        FollowerConstants.forwardZeroPowerAcceleration = -35.1168;
        FollowerConstants.lateralZeroPowerAcceleration = -46.3753;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.5,0.00001,0.01,0);
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0.00001,0.01,0.02);
        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.012, 0.0, 0.0001, 0.6, 0.0);
    }
}
