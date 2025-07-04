package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoTransfer {
    private enum State {
        IDLE,
        RETRACT_HOR_SLIDE,
        RETRACT_VERT_SLIDE,
        CLOSE_OUT_CLAW,
        LIFT_VERT_SLIDE,
        ROTATE_OUT_ARM,
        DONE
    }

    private State currentState = State.IDLE;
    private long stateStartTime;

    private final Servo outClaw;
    private final DcMotor verticalSlide;
    private final DcMotor horizontalSlide;
    private final Servo outRotation;

    public AutoTransfer(HardwareMap hardwareMap) {
        outClaw = hardwareMap.get(Servo.class, "OutClaw");
        verticalSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
        outRotation = hardwareMap.get(Servo.class, "OutRotation");
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startTransfer() {
        if (currentState == State.IDLE || currentState == State.DONE) {
            currentState = State.RETRACT_HOR_SLIDE;
            stateStartTime = System.currentTimeMillis();
        }
    }

    public void update() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        switch (currentState) {
            case RETRACT_HOR_SLIDE:
                horizontalSlide.setTargetPosition(0);
                horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlide.setPower(0.5);
                if (elapsed > 500) {
                    currentState = State.RETRACT_VERT_SLIDE;
                    stateStartTime = System.currentTimeMillis();
                }
                break;
            case RETRACT_VERT_SLIDE:
                verticalSlide.setTargetPosition(0);
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(0.5);
                if (elapsed > 500) {
                    currentState = State.CLOSE_OUT_CLAW;
                    stateStartTime = System.currentTimeMillis();
                }
                break;
            case CLOSE_OUT_CLAW:
                outClaw.setPosition(0.35);
                if (elapsed > 300) {
                    currentState = State.LIFT_VERT_SLIDE;
                    stateStartTime = System.currentTimeMillis();
                }
                break;
            case LIFT_VERT_SLIDE:
                verticalSlide.setTargetPosition(800); // set your target height
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(0.5);
                if (elapsed > 500) {
                    currentState = State.ROTATE_OUT_ARM;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case ROTATE_OUT_ARM:
                outRotation.setPosition(0.5); // rotate to outtake position
                currentState = State.DONE;
                break;

            case DONE:
                // Optional: reset if needed
                break;

            case IDLE:
                // Waiting for start
                break;
        }
    }

    public boolean isBusy() {
        return currentState != State.IDLE && currentState != State.DONE;
    }

    public void reset() {
        currentState = State.IDLE;
    }
}
