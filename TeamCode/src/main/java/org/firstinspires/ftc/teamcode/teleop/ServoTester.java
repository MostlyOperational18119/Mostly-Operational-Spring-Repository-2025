package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTester")
public class ServoTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double Position = 0;
        Servo dropperServo = hardwareMap.servo.get("OutClaw");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                Position += 0.01;
            }
            if (gamepad1.bWasPressed()) {
                Position -= 0.01;
            }

            telemetry.addData("position", Position);
            telemetry.update();
            dropperServo.setPosition(Position);
        }
    }
}
//outswivel: 0.17
//outrotation: 1
//0.57 inRotation down
//0.04 inRotation up
//0.18 InSwivel
//0.84