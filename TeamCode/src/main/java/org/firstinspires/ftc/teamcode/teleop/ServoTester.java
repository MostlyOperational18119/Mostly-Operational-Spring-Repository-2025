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
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Servo dropperServo = hardwareMap.servo.get("dropperServo");

        waitForStart();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            if (gamepad1.a && !previousGamepad1.a) {
                Position += 0.01;
            }
            if (gamepad1.b && !previousGamepad1.b) {
                Position -= 0.01;
            }

            telemetry.addData("position", Position);
            telemetry.update();
            dropperServo.setPosition(Position);
        }
    }
}
