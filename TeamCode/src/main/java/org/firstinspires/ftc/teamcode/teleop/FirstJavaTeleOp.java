package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

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



        DcMotor verticalSlide = hardwareMap.dcMotor.get("verticalSlide");
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor horizontalSlide = hardwareMap.dcMotor.get("horizontalSlide");
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad driverCurrent = gamepad1;
        Gamepad driverPrevious = gamepad1;
        Gamepad gunnerCurrent = gamepad2;
        Gamepad gunnerPrevious = gamepad2;

        float driverLeftX = gamepad1.left_stick_x;
        float driverLeftY = -gamepad1.left_stick_y;
        float driverRightX = gamepad1.right_stick_x;

        float gunnerRightY = gamepad2.right_stick_y;
        boolean gunnerRightDown = gamepad2.right_stick_button;
        float gunnerLeftX = gamepad2.left_stick_x;
        boolean gunnerLeftDown = gamepad2.left_stick_button;

        double speedDiv = 1.0;

        telemetry.addLine("Init done");
        telemetry.update();

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            driverCurrent.copy(driverPrevious);
            gamepad1.copy(driverCurrent);

            gunnerCurrent.copy(gunnerPrevious);
            gamepad2.copy(gunnerCurrent);

            driverLeftX = gamepad1.left_stick_x;
            driverLeftY = -gamepad1.left_stick_y;
            driverRightX = gamepad1.right_stick_x;
            gunnerRightY = gamepad2.right_stick_y;
            gunnerLeftX = gamepad2.left_stick_x;

            LLResult result = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();
            double[] out = new double[] {};
            if (result != null) {
                out = result.getPythonOutput();
            } else {
                telemetry.addLine("Warning: LimeLight output is null");
            }

            motorFL.setPower((driverLeftY + driverLeftX + driverRightX) / speedDiv);
            motorFR.setPower((driverLeftY - driverLeftX - driverRightX) / speedDiv);
            motorBL.setPower((driverLeftY - driverLeftX + driverRightX) / speedDiv);
            motorBR.setPower((driverLeftY + driverLeftX - driverRightX) / speedDiv);
            verticalSlide.setPower((gunnerRightY >= 0.0) ? gunnerRightY : 1.0);
            horizontalSlide.setPower(gunnerLeftX);


            telemetry.addLine("Running");
            telemetry.addLine(String.format(Locale.ENGLISH, "Speed div: %f", speedDiv));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor FL: %f", motorFL.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor FR: %f", motorFR.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor BL: %f", motorBL.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor BR: %f", motorBR.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Right X: %f", gamepad1.right_stick_x));
            telemetry.addLine(String.format(Locale.ENGLISH, "Limelight Output: %s", Arrays.toString(out)));
            telemetry.addLine(String.format(Locale.ENGLISH, "Limelight Status: %s", status.toString()));
            telemetry.update();
        }
    }
}