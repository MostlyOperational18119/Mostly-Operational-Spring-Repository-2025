package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

@TeleOp(name = "Program1")
public class turnmotor extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        double leftY;
        double leftX;
        double rightX;
        double speed = 2.0;
        waitForStart();
       while (opModeIsActive()) {
           leftY = -gamepad1.left_stick_y;
           leftX = gamepad1.left_stick_x;
           rightX = gamepad1.right_stick_x;
           motorFL.setPower(Math.atan(1.12*((leftY+leftX+rightX)))/speed);
           motorBL.setPower(Math.atan(1.12*((leftY-leftX+rightX)))/speed);
           motorFR.setPower(Math.atan(1.12*((leftY-leftX-rightX)))/speed);
           motorBR.setPower(Math.atan(1.12*((leftY+leftX-rightX)))/speed);
           telemetry.update();
           if (gamepad1.left_stick_button || gamepad1.right_trigger >= 0.5) {
               speed = 1.0;
           } else if (gamepad1.left_trigger >= .5) {
               speed = 3.0;
           } else {
               speed = 2.0;
           }

       }
    }
}