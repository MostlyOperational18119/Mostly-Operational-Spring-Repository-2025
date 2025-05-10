package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Locale;

@TeleOp(name = "First Java TeleOp")
public class FirstJavaTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

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
        float gunnerLeftX = gamepad2.left_stick_x;

        double speedDiv = 1.0;

        telemetry.addLine("Init done");
        telemetry.update();

        waitForStart();

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

            motorFL.setPower((driverLeftY + driverLeftX + driverRightX) / speedDiv);
            motorFR.setPower((driverLeftY - driverLeftX - driverRightX) / speedDiv);
            motorBL.setPower((driverLeftY - driverLeftX + driverRightX) / speedDiv);
            motorBR.setPower((driverLeftY + driverLeftX - driverRightX) / speedDiv);

            telemetry.addLine("Running");
            telemetry.addLine(String.format(Locale.ENGLISH, "Speed div: %f", speedDiv));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor FL: %f", motorFL.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor FR: %f", motorFR.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor BL: %f", motorBL.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor BR: %f", motorBR.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Right X: %f", gamepad1.right_stick_x));
            telemetry.update();
        }
    }
}
