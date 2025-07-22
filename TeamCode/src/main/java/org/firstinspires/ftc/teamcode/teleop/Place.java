package org.firstinspires.ftc.teamcode.teleop;

import android.view.SearchEvent;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Place {
    private enum State {
        SLIDE_DOWN,
        RELEASE,
        DONE;
    }

    private State currentState = State.DONE;
    private long stateStartTime;

    private final Servo outClaw;
    private final Servo outRotation;
    private final DcMotorEx verticalSlide;
    private final Telemetry telemetry;
    private boolean hasReset = false;

    public Place(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        outClaw = hardwareMap.get(Servo.class, "OutClaw");
        outRotation = hardwareMap.get(Servo.class, "OutRotation");
        verticalSlide = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void start() {
        if (currentState == State.DONE) {
            currentState = State.SLIDE_DOWN;
            stateStartTime = System.currentTimeMillis();
        }
    }

    public void update() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        switch (currentState) {
            case SLIDE_DOWN:
                verticalSlide.setTargetPosition(800);
                outRotation.setPosition(0.0);
                verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(0.5);
                if (elapsed > 300) {
                    currentState = State.RELEASE;
                }
                break;
            case RELEASE:
                if (!hasReset) {
                    hasReset = true;
                    outClaw.setPosition(0.15);
                }
                hasReset = false;
                verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                currentState = State.DONE;
                break;
            case DONE:
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
