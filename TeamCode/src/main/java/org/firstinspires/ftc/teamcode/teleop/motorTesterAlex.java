package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motorTesting")
public class motorTesterAlex extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class,"motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.dpadUpWasPressed()) {

            }


        }

    }

}
