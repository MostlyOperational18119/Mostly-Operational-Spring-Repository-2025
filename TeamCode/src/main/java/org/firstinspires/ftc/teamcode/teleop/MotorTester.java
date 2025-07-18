package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Motor Tester")
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor horSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
        horSlide.setTargetPosition(0);
        horSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                horSlide.setTargetPosition(horSlide.getTargetPosition()-25);
            }

            if (gamepad1.yWasPressed()) {
                horSlide.setTargetPosition(horSlide.getTargetPosition()+25);
            }

            horSlide.setPower(1.0);

            telemetry.addLine("Running");
            telemetry.addLine("Encoder Position: " + horSlide.getCurrentPosition());
            telemetry.addData("Target Position:", horSlide.getTargetPosition());
            telemetry.addData("Motor speed:", horSlide.getPower());
            telemetry.update();

            sleep(50);
        }
    }
}