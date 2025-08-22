package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoTesterAlex")
public class servotesteralex extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, "openClose");
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                servo.setPosition(servo.getPosition() + 0.01);
            }

            if (gamepad1.dpadDownWasPressed()) {
                servo.setPosition(servo.getPosition() - 0.01);
            }
            telemetry.addData("servoPosition", servo.getPosition());
            telemetry.update();
        }
    }


}
