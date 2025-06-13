package org.firstinspires.ftc.teamcode.services.Communication;

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

    // Plan mode

    public Double forwardSpeed;
    public Double sidewaysSpeed;
    public Double rotationSpeed;

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

        this.forwardSpeed = null;
        this.sidewaysSpeed = null;
        this.rotationSpeed = null;
    }

    public DriveServiceInput(Double forwardSpeed, Double sidewaysSpeed, Double rotationSpeed) {
        mode = DriveServiceInputMode.PLAN;

        this.flSpeed = null;
        this.frSpeed = null;
        this.blSpeed = null;
        this.brSpeed = null;
        this.verticalSlideSpeed = null;
        this.horizontalSlideSpeed = null;

        this.forwardSpeed = forwardSpeed;
        this.sidewaysSpeed = sidewaysSpeed;
        this.rotationSpeed = rotationSpeed;
    }
}
