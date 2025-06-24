package org.firstinspires.ftc.teamcode.teleop;

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

import java.util.Arrays;
import java.util.Locale;

@TeleOp(name = "First Java TeleOp")
public class FirstJavaTeleOp extends LinearOpMode {

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

        DcMotor horizontalSlide = hardwareMap.dcMotor.get("horizontalSlide");
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Servo inRotation = hardwareMap.servo.get("InRotation");
        inRotation.setPosition(0.81);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad driverCurrent = new Gamepad();
        Gamepad driverPrevious = new Gamepad();
        Gamepad gunnerCurrent = new Gamepad();
        Gamepad gunnerPrevious = new Gamepad();

        float driverLeftX;
        float driverLeftY;
        float driverRightX;

        float gunnerRightY;
        float gunnerLeftY;
        float gunnerRightTrigger;
        float gunnerLeftTrigger;

        double speedDiv = 1.0;
        int vertTarget = 0;
        double P;
        double D;
        double feedforward;
        int vertTop = 500;

        telemetry.addLine("Init done");
        telemetry.update();

        waitForStart();

        limelight.start();


        while (opModeIsActive()) {

            driverLeftX = gamepad1.left_stick_x;
            driverLeftY = -gamepad1.left_stick_y;
            driverRightX = gamepad1.right_stick_x;

            gunnerRightY = -gamepad2.right_stick_y;
            gunnerLeftY = gamepad2.left_stick_y;
            gunnerRightTrigger = gamepad2.right_trigger;
            gunnerLeftTrigger = gamepad2.left_trigger;

            gunnerPrevious.copy(gunnerCurrent);
            driverCurrent.copy(driverPrevious);

            gunnerCurrent.copy(gamepad2);
            driverCurrent.copy(gamepad1);

            LLResult result = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();
            double[] out = new double[] {};
            if (result != null) {
                out = result.getPythonOutput();
            } else {
                telemetry.addLine("Warning: LimeLight output is null");
            }

            motorFL.setPower(Math.atan(1.12*(driverLeftY + driverLeftX + driverRightX)) / speedDiv);
            motorFR.setPower(Math.atan(1.12*(driverLeftY - driverLeftX - driverRightX)) / speedDiv);
            motorBL.setPower(Math.atan(1.12*(driverLeftY - driverLeftX + driverRightX)) / speedDiv);
            motorBR.setPower(Math.atan(1.12*(driverLeftY + driverLeftX - driverRightX)) / speedDiv);

            double error = vertTarget - verticalSlide.getCurrentPosition();

            //adjusts the PD values when the slide is going down
            if (verticalSlide.getCurrentPosition() > vertTarget) {
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

            double derivative = -verticalSlide.getVelocity();
            double verticalPower = P*error + D*derivative;

            if (gunnerCurrent.dpad_down && !gunnerPrevious.dpad_down) {
                vertTarget = 0;
            } else if (gunnerCurrent.dpad_up && !gunnerPrevious.dpad_up) {
                vertTarget = vertTop;
            }

            verticalSlide.setPower(verticalPower + feedforward);

            vertTarget = vertTarget + (int) (10*gunnerRightY);

            if (gunnerCurrent.left_bumper && !gunnerPrevious.left_bumper) {
                vertTop -= 250;
            } else if (gunnerCurrent.right_bumper && !gunnerPrevious.right_bumper) {
                vertTop += 250;
            }

            horizontalSlide.setPower(gunnerLeftY);

            if (gunnerRightTrigger > 0){
                intakeMotor.setPower(gunnerRightTrigger);
            } else {
                intakeMotor.setPower(-gunnerLeftTrigger);
            }

            telemetry.addLine("Running");
            telemetry.addData("vertical slide target pos: ", vertTarget);
            telemetry.addData("error", error);
            telemetry.addData("vertTop: ", vertTop);
            telemetry.addData("vert speed: ", verticalSlide.getVelocity());
            telemetry.addData("vertical encoder: ", verticalSlide.getCurrentPosition());
            telemetry.addData("Speed div: ", speedDiv);
            telemetry.addData("Motor FL: ", motorFL.getPower());
            telemetry.addData("Motor FR: ", motorFR.getPower());
            telemetry.addData("Motor BL: ", motorBL.getPower());
            telemetry.addData("Motor BR: ", motorBR.getPower());
            telemetry.addData("Right X: ", gamepad1.right_stick_x);
            telemetry.addData("Limelight Output: ", Arrays.toString(out));
            telemetry.addData("Limelight Status: ", status.toString());
            telemetry.update();
        }
    }
}
