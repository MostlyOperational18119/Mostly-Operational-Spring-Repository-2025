package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoTransfer {
    private enum State {
        IDLE,
        RETRACT_HOR_SLIDE,
        CLOSE_CLAW,
        WAIT_AFTER_CLAW,
        LIFT_SLIDE,
        WAIT_AFTER_LIFT,
        ROTATE_ARM,
        DONE
    }

    private State currentState = State.IDLE;
    private long stateStartTime;

    private final Servo claw;
    private final DcMotor slide;
    private final Servo arm;

    public AutoTransfer(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "OutClaw");
        slide = hardwareMap.get(DcMotor.class, "verticalSlide");
        arm = hardwareMap.get(Servo.class, "OutRotation");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            case CLOSE_CLAW:
                claw.setPosition(0.35);
                if (elapsed > 300) {
                    currentState = State.LIFT_SLIDE;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case LIFT_SLIDE:
                slide.setTargetPosition(800); // set your target height
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1.0);
                if (elapsed > 500) {
                    currentState = State.ROTATE_ARM;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case ROTATE_ARM:
                arm.setPosition(0.5); // rotate to outtake position
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
