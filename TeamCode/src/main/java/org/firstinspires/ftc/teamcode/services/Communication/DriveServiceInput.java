package org.firstinspires.ftc.teamcode.services.Communication;

public class DriveServiceInput {
    public enum DriveServiceInputMode {
        MANUAL,
        PLAN
    }

    public final DriveServiceInputMode mode;

    public Double flSpeed;
    public Double frSpeed;
    public Double blSpeed;
    public Double brSpeed;

    public DriveServiceInput() {
        mode = DriveServiceInputMode.PLAN;
    }

    public DriveServiceInput(Double flSpeed, Double frSpeed, Double blSpeed, Double brSpeed) {
        mode = DriveServiceInputMode.MANUAL;

        this.flSpeed = flSpeed;
        this.frSpeed = frSpeed;
        this.blSpeed = blSpeed;
        this.brSpeed = brSpeed;
    }
}
