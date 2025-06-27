package org.firstinspires.ftc.teamcode.services.Communication;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.stream.Stream;

public class DriveServiceInput {
    public enum DriveServiceInputMode {
        MANUAL,
        PLAN
    }

    public final DriveServiceInputMode mode;

    // Manual mode
    public Double flSpeed;
    public Double frSpeed;
    public Double blSpeed;
    public Double brSpeed;
    public Double verticalSlideSpeed;
    public Double horizontalSlideSpeed;
    public Boolean doBreak;

    // Plan mode

    public double[] control;

    public DriveServiceInput() {
        mode = DriveServiceInputMode.PLAN;
    }

    public DriveServiceInput(Double flSpeed, Double frSpeed, Double blSpeed, Double brSpeed, Double verticalSlideSpeed, Double horizontalSlideSpeed) {
        mode = DriveServiceInputMode.MANUAL;

        this.flSpeed = flSpeed;
        this.frSpeed = frSpeed;
        this.blSpeed = blSpeed;
        this.brSpeed = brSpeed;
        this.verticalSlideSpeed = verticalSlideSpeed;
        this.horizontalSlideSpeed = horizontalSlideSpeed;
        this.doBreak = false;

        this.control = null;
    }

    public DriveServiceInput(Boolean doBreak) {
        mode = DriveServiceInputMode.MANUAL;

        this.flSpeed = 0.0;
        this.frSpeed = 0.0;
        this.blSpeed = 0.0;
        this.brSpeed = 0.0;
        this.verticalSlideSpeed = 0.0;
        this.horizontalSlideSpeed = 0.0;
        this.doBreak = doBreak;

        this.control = null;
    }

    public DriveServiceInput(double[] control) {
        mode = DriveServiceInputMode.PLAN;

        this.flSpeed = null;
        this.frSpeed = null;
        this.blSpeed = null;
        this.brSpeed = null;
        this.verticalSlideSpeed = null;
        this.horizontalSlideSpeed = null;
        this.doBreak = null;

        this.control = control;
    }
}
