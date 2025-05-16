package org.firstinspires.ftc.teamcode.services.Drive;

public class DriveServiceInput {
    public enum DriveServiceInputMode {
        MANUAL,
        PLAN
    }

    public final DriveServiceInputMode mode;

    public DriveServiceInput() {
        mode = DriveServiceInputMode.PLAN;
    }

    public DriveServiceInput(Double flSpeed, Double frSpeed, Double blSpeed, Double brSpeed) {
        mode = DriveServiceInputMode.MANUAL;
    }
}
