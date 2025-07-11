package org.firstinspires.ftc.teamcode.teleop;

import android.view.SearchEvent;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoTransfer {
    private enum State {
        RESET_SLIDES,
        CLOSE_OUT_CLAW,
        LIFT_VERT_SLIDE,
        ROTATE_OUT_ARM,
        DONE;
    }

    private State currentState = State.DONE;
    private long stateStartTime;
    private long horRetractTime;

    private final Servo outClaw;
    private final DcMotor verticalSlide;
    private final DcMotor horizontalSlide;
    private final DcMotor intakeMotor;
    private final Servo outRotation;
    private final Servo outSwivel;
    private final Servo inRotation;
    private final Telemetry telemetry;
    private boolean hasReset = false;

    public AutoTransfer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        outClaw = hardwareMap.get(Servo.class, "OutClaw");
        verticalSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
        outRotation = hardwareMap.get(Servo.class, "OutRotation");
        outSwivel = hardwareMap.get(Servo.class, "OutSwivel");
        inRotation = hardwareMap.get(Servo.class, "InRotation");
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    }

    public void startTransfer() {
        if (currentState == State.DONE) {
            currentState = State.RESET_SLIDES;
            stateStartTime = System.currentTimeMillis();
        }
    }

    public void update() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        switch (currentState) {
            case RESET_SLIDES:
                verticalSlide.setTargetPosition(0);
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(0.5);
                outSwivel.setPosition(0.17);
                outRotation.setPosition(0.98);
                outClaw.setPosition(0.4);
                inRotation.setPosition(0.04);
                horizontalSlide.setTargetPosition(0);
                horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlide.setPower(1);
                intakeMotor.setPower(1);
                telemetry.addData("should switch continue", verticalSlide.getCurrentPosition() <= 100 && horizontalSlide.getCurrentPosition() <= 100);
                if (verticalSlide.getCurrentPosition() <= 100 && horizontalSlide.getCurrentPosition() <= 100) {
                    if (!hasReset) {
                        hasReset = true;
                        stateStartTime = System.currentTimeMillis();
                    }
                    telemetry.addData("elapsed", elapsed);
                    if (elapsed > 1000) {
                        currentState = State.CLOSE_OUT_CLAW;
                    }
                }
                break;
            case CLOSE_OUT_CLAW:
                outClaw.setPosition(0.51);
                if (elapsed > 600) {
                    currentState = State.LIFT_VERT_SLIDE;
                    stateStartTime = System.currentTimeMillis();
                    intakeMotor.setPower(0);
                }
                break;
            case LIFT_VERT_SLIDE:
                verticalSlide.setTargetPosition(1600); // set your target height
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(0.5);
                if (elapsed > 500) {
                    currentState = State.ROTATE_OUT_ARM;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case ROTATE_OUT_ARM:
                outRotation.setPosition(0.29); // rotate to outtake position
                currentState = State.DONE;
                break;

            case DONE:
                // Optional: reset if needed
                break;
        }
    }

    public boolean isBusy() {
        return currentState != State.DONE;
    }

    public State returnState() {
        return currentState;
    }
}
