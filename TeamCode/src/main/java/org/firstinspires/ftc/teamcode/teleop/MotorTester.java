package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Motor Tester")
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() {

        DcMotor vertSlide = hardwareMap.get(DcMotor.class, "vertSlide");
        waitForStart();

        Gamepad driverCurrent = gamepad1;
        Gamepad driverPrevious = gamepad1;
        Gamepad gunnerCurrent = gamepad2;
        Gamepad gunnerPrevious = gamepad2;


        while (opModeIsActive()) {

            driverCurrent.copy(driverPrevious);
            gamepad1.copy(driverCurrent);

            gunnerCurrent.copy(gunnerPrevious);
            gamepad2.copy(gunnerCurrent);

            Boolean up = gamepad1.y;
            Boolean down = gamepad1.a;

            vertSlide.setPower(up ? 0.1 : (down ? -0.1 : 0.0));

            telemetry.addLine("Running");
            telemetry.addLine("Encoder Position: " + vertSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}