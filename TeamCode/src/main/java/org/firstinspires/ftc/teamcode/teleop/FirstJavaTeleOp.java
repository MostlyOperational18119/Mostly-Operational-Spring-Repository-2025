package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Path;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@TeleOp(name = "First Java TeleOp")
public class FirstJavaTeleOp extends LinearOpMode {
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

    public void horizontalSlideTo(DcMotorEx slide, Integer target) {
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

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        DcMotorEx verticalSlide = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide");
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-170, 0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

        Servo inRotation = hardwareMap.servo.get("InRotation");
        Servo outRotation = hardwareMap.servo.get("OutRotation");
        inRotation.setPosition(0.1);
        Servo outClaw = hardwareMap.get(Servo.class, "OutClaw");
        Servo inStop = hardwareMap.get(Servo.class, "InStop");
        Servo outSwivel = hardwareMap.get(Servo.class, "OutSwivel");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad driverCurrent = gamepad1;
        Gamepad driverPrevious = gamepad1;
        Gamepad gunnerCurrent = gamepad2;
        Gamepad gunnerPrevious = gamepad2;

        AutoTransfer transfer = new AutoTransfer(hardwareMap, telemetry);
        Place place = new Place(hardwareMap,telemetry);

        float driverLeftX;
        float driverLeftY;
        float driverRightX;

        float gunnerLeftY;
        float gunnerRightY;
        float gunnerRightTrigger;
        float gunnerLeftTrigger;
        boolean gunnerLB;
        double speedDiv = 1.0;
        double currentSide = 0.29;
        double outPosition = 0.96;
        double MoveMulti = 1;
        long startTime = 0;
        int vertTarget = 0;
        int horizontalTarget = 0;
        boolean horizontalIsManual = true;
        boolean Open = false;
        boolean IntakeUp = false;

        telemetry.addLine("Init done");
        telemetry.update();

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            pinpoint.update();

            driverLeftX = (float) (gamepad1.left_stick_x * MoveMulti);
            driverLeftY = -gamepad1.left_stick_y;
            driverRightX = gamepad1.right_stick_x;

            gunnerLeftY = gamepad2.left_stick_y;
            gunnerRightY = gamepad2.right_stick_y;
            gunnerLB = gamepad2.left_bumper;
            gunnerRightTrigger = gamepad2.right_trigger;
            gunnerLeftTrigger = gamepad2.left_trigger;

            LLResult result = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();
            double[] out = new double[] {};
            if (result != null) {
                out = result.getPythonOutput();
            } else {
                telemetry.addLine("Warning: LimeLight output is null");
            }

            motorFL.setPower((Math.atan(1.12*(driverLeftY + driverLeftX + driverRightX)) / speedDiv) * MoveMulti);
            motorFR.setPower((Math.atan(1.12*(driverLeftY - driverLeftX - driverRightX)) / speedDiv) * MoveMulti);
            motorBL.setPower((Math.atan(1.12*(driverLeftY - driverLeftX + driverRightX)) / speedDiv) * MoveMulti);
            motorBR.setPower((Math.atan(1.12*(driverLeftY + driverLeftX - driverRightX)) / speedDiv) * MoveMulti);

            //horizontal slide controls?
            if (gunnerLeftY != 0) {
                horizontalIsManual = true;
                //horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.aWasPressed()) {
                MoveMulti *= -1;
            }

            if (gunnerRightY != 0 && !transfer.isBusy() && !place.isBusy()) {
                vertTarget -= (int) (gunnerRightY * 25);
            }

            if (horizontalIsManual && !transfer.isBusy() && !place.isBusy()) {
                horizontalSlide.setPower(-gunnerLeftY);
            }

            if (gunnerRightTrigger > 0) {
                intakeMotor.setPower(gunnerRightTrigger);
                inRotation.setPosition(0.89);
            } else if (gunnerLeftTrigger > 0) {
                intakeMotor.setPower(-gunnerLeftTrigger);
                inRotation.setPosition(0.89);
            } else {
                intakeMotor.setPower(0);
            }

            if (horizontalSlide.getCurrentPosition() <= 100) {
                inStop.setPosition(0.37);
                inRotation.setPosition(0.29);
                IntakeUp = true;
            } else {
                inStop.setPosition(0.86);
            }

            if (gamepad2.aWasPressed() && !transfer.isBusy()) {
                transfer.start(outPosition);
                Open = false;
                vertTarget = 1950;
            }
            if (gamepad2.xWasPressed() && !place.isBusy()) {
                place.start();
            }

            if (gamepad2.yWasPressed()) {
                if (IntakeUp) {
                    inRotation.setPosition(0.25);
                } else {
                    inRotation.setPosition(0.89);
                }
                IntakeUp = !IntakeUp;
            }

            if (gamepad2.rightBumperWasPressed()) {
                if (Open) {
                    outClaw.setPosition(0.26);
                    Open = false;
                } else {
                    outClaw.setPosition(0.15);
                    Open = true;
                }
            }

            if (gamepad2.bWasPressed()) {
                if (currentSide == 0.29) {
                    outSwivel.setPosition(0.17);
                    currentSide = 0.58;
                    outPosition = 0.58;
                } else {
                    outSwivel.setPosition(0.85);
                    currentSide = 0.29;
                    outPosition = 0.29;
                }
            }
            if (gamepad2.dpadDownWasPressed()) {
                vertTarget = 0;
                outPosition = 0.16;
            }

            if (gamepad2.dpadUpWasPressed()) {
                vertTarget = 1950;
                outPosition = currentSide;
            }

            if(gamepad2.leftStickButtonWasPressed()) {
                horizontalIsManual = false;
                horizontalTarget = 0;
            }

            if (!transfer.isBusy() && !place.isBusy()) {
                outRotation.setPosition(outPosition);
                verticalSlideTo(verticalSlide, vertTarget);
            }

            if (gamepad2.dpadLeftWasPressed()) {
                vertTarget = 1200;
                outPosition = 0.12;
            }

            driverPrevious.copy(driverCurrent);

            driverCurrent.copy(gamepad1);

            gunnerPrevious.copy(gunnerCurrent);
            gunnerCurrent.copy(gamepad2);
            transfer.update();
            place.update();

            telemetry.addLine("Running");
            telemetry.addData("Transfer State", transfer.returnState());
            telemetry.addData("vertical encoder: ", verticalSlide.getCurrentPosition());
            telemetry.addData("vertical target: ", verticalSlide.getTargetPosition());
            telemetry.addData("vertical mode: ", verticalSlide.getMode());
            telemetry.addData("outPosition: ", outPosition);
            telemetry.addData("horizontal encoder: ", horizontalSlide.getCurrentPosition());
            telemetry.addData("horizontal manual mode: ", horizontalIsManual);
            telemetry.addData("horizontal target: ", horizontalTarget);
            telemetry.addData("gunnerLB: ", gunnerLB);
            telemetry.addData("Right X: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
