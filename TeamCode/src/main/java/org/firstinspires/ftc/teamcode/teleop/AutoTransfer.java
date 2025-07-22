package org.firstinspires.ftc.teamcode.teleop;

import android.view.SearchEvent;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoTransfer {
    private enum State {
        SWIVEL_FIRST,
        RESET_SLIDES,
        CLOSE_OUT_CLAW,
        LIFT_VERT_SLIDE,
        ROTATE_OUT_ARM,
        DONE;
    }

    public void horizontalSlideTo(DcMotor slide, Integer target) {
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (slide.getCurrentPosition() > target + 10) {
            slide.setPower(-1.0);
        } else if (slide.getCurrentPosition() < target - 10) {
            slide.setPower(1.0);
        } else {
            slide.setPower(0);
        }
    }

    public void verticalSlideTo(DcMotorEx slide, Integer target) {
        double P;
        double D;
        double feedforward;

        double error = target - slide.getCurrentPosition();

        if (slide.getCurrentPosition() > target) {
            P = 0.004;
            D = 0.0;
            if (Math.abs(error) < 50) {
                feedforward = 0.0;
            } else {
                feedforward = 0.11;
            }
        } else {
            P = 0.03;
            D = 0.0002;
            feedforward = 0.0;
        }

        double derivative = -slide.getVelocity();
        double verticalPower = P * (target - slide.getCurrentPosition()) + D * derivative;

        slide.setPower(verticalPower + feedforward);
    }

    private State currentState = State.DONE;
    private long stateStartTime;
    private long horRetractTime;

    private final Servo outClaw;
    private final DcMotorEx verticalSlide;
    private final DcMotorEx horizontalSlide;
    private final DcMotor intakeMotor;
    private final Servo outRotation;
    private final Servo outSwivel;
    private final Servo inRotation;
    private final Servo inStop;
    private final Telemetry telemetry;
    private boolean hasReset = false;
    private boolean hasReset2 = false;

    private double outPosition;

    public AutoTransfer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        outClaw = hardwareMap.get(Servo.class, "OutClaw");
        verticalSlide = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide");
        outRotation = hardwareMap.get(Servo.class, "OutRotation");
        outSwivel = hardwareMap.get(Servo.class, "OutSwivel");
        inRotation = hardwareMap.get(Servo.class, "InRotation");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        inStop = hardwareMap.get(Servo.class, "InStop");
    }

    public void start(double outPosition) {
        if (currentState == State.DONE) {
            currentState = State.SWIVEL_FIRST;
            stateStartTime = System.currentTimeMillis();
            this.outPosition = outPosition;
        }
    }

    public void update() {
        long elapsed = System.currentTimeMillis() - stateStartTime;

        switch (currentState) {
            case SWIVEL_FIRST:
                verticalSlide.setTargetPosition(400);
                verticalSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(1);
                outSwivel.setPosition(0.85); //previously 0.17
                outRotation.setPosition(0.96);
                if (elapsed > 500) {
                    currentState = State.RESET_SLIDES;
                    stateStartTime = System.currentTimeMillis();
                }
                break;
            case RESET_SLIDES:
                verticalSlide.setTargetPosition(0);
                verticalSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                verticalSlide.setPower(1);
                outClaw.setPosition(0.15);
                inRotation.setPosition(0.3);
                horizontalSlideTo(horizontalSlide,0);
                intakeMotor.setPower(1);
//                telemetry.addData("should switch continue", verticalSlide.getCurrentPosition() <= 100 && horizontalSlide.getCurrentPosition() <= 100);
                if (verticalSlide.getCurrentPosition() <= 100 && horizontalSlide.getCurrentPosition() <= 100) {
                    if (!hasReset) {
                        hasReset = true;
                        stateStartTime = System.currentTimeMillis();
                        inStop.setPosition(0.37);
                    }
//                    telemetry.addData("elapsed", elapsed);
                    if (elapsed > 700) {
                        hasReset = false;
                        currentState = State.CLOSE_OUT_CLAW;
                        stateStartTime = System.currentTimeMillis();
                    }
                }
                break;
            case CLOSE_OUT_CLAW:
                    outClaw.setPosition(0.26);
                if (elapsed > 1000) {
                    currentState = State.LIFT_VERT_SLIDE;
                    stateStartTime = System.currentTimeMillis();
                    intakeMotor.setPower(0);
                }
                break;
            case LIFT_VERT_SLIDE:
                    verticalSlide.setTargetPosition(1950); // set your target height
                verticalSlide.setPower(1);
                    verticalSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                if (elapsed > 500) {
                    currentState = State.ROTATE_OUT_ARM;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case ROTATE_OUT_ARM:
                outRotation.setPosition(outPosition); // rotate to outtake position
                horizontalSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                verticalSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
