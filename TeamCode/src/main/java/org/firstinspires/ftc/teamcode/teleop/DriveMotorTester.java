package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Locale;

@TeleOp(name = "Drive Motor Tester")
public class DriveMotorTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

        Gamepad driverCurrent = gamepad1;
        Gamepad driverPrevious = gamepad1;

        telemetry.addLine("Init done");
        telemetry.update();
        boolean uh = false;

        waitForStart();

        while (opModeIsActive()) {
            driverCurrent.copy(driverPrevious);
            gamepad1.copy(driverCurrent);

            telemetry.addLine("A motorFL, B motorFR, X motorBL, Y motorBR");

            if (driverCurrent.a && !driverPrevious.a) {
                telemetry.addLine("EEEE");
                uh = true;
                motorFL.setPower((motorFL.getPower() == 1.0) ? 0.0 : 1.0);
            }

            if (driverCurrent.b && !driverPrevious.b) {
                motorFR.setPower((motorFR.getPower() == 1.0) ? 0.0 : 1.0);
            }

            if (driverCurrent.x && !driverPrevious.x) {
                motorBL.setPower((motorBL.getPower() == 1.0) ? 0.0 : 1.0);
            }

            if (driverCurrent.y && !driverPrevious.y) {
                motorBR.setPower((motorBR.getPower() == 1.0) ? 0.0 : 1.0);
            }

            telemetry.addLine("driverCurrent.a: " + driverCurrent.a);
            telemetry.addLine("driverPrevious.a: " + driverPrevious.a);
            telemetry.addLine("uh: " + uh);
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor FL: %f", motorFL.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor FR: %f", motorFR.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor BL: %f", motorBL.getPower()));
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor BR: %f", motorBR.getPower()));

            telemetry.update();
        }
    }
}
