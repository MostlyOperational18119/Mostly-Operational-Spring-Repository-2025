package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;

public class FConstants {
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 14.515; // In kg
    }
}
