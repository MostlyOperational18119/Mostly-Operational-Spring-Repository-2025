package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class PicklebotCodeRecreation extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");

        float driverLeftY;
        float driverLeftX;
        float driverRightY;
        float driverRightX;
        float open = 0.52f;
        float close = 0.4f;
        double speedDiv = 2.0;
        float spooled = 0.19f;
        float unspooled = 0.04f;

        int rotationTarget = 0;
        Servo spool = hardwareMap.servo.get("spool");
        Servo openClose = hardwareMap.servo.get("openClose");

        DcMotor Rotation = hardwareMap.get(DcMotor.class, "Rotation");
        DcMotor spin1 = hardwareMap.get(DcMotor.class, "flywheelBottom");
        DcMotor spin2 = hardwareMap.get(DcMotor.class, "flywheelRight");
        DcMotor spin3 = hardwareMap.get(DcMotor.class, "flywheelLeft");
        spin3.setDirection(DcMotorSimple.Direction.REVERSE);


        Rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotation.setTargetPosition(0);
        Rotation.setPower(0.5);
        Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //settings
        int rotationMax = 1000;
        int rotationMin = -1000;

        waitForStart();

        while (opModeIsActive()) {
            //drive code

            driverLeftY = gamepad1.left_stick_y;
            driverLeftX = -gamepad1.left_stick_x;
            driverRightY = gamepad1.right_stick_y;
            driverRightX = gamepad1.right_stick_x;

            motorFL.setPower(-(driverLeftY + driverLeftX + driverRightX) / speedDiv);
            motorFR.setPower(-(driverLeftY - driverLeftX - driverRightX) / speedDiv);
            motorBL.setPower(-(driverLeftY - driverLeftX + driverRightX) / speedDiv);
            motorBR.setPower(-(driverLeftY + driverLeftX - driverRightX) / speedDiv);

            //rotation code

            Rotation.setTargetPosition(rotationTarget);
            Rotation.setPower(0.4);

                    if (gamepad1.dpadUpWasPressed() && rotationTarget < rotationMax) {
                        rotationTarget += 50;
                    }
                    if (gamepad1.dpadDownWasPressed() && rotationTarget > rotationMin) {
                        rotationTarget -= 50;
                    }

            if (gamepad1.yWasPressed()) {
                spin1.setPower(0.5);
                spin2.setPower(0.5);
                spin3.setPower(0.5);
            }

            if (gamepad1.bWasPressed()) {
                openClose.setPosition(open);
                spool.setPosition(spooled);
                sleep(2000);
                openClose.setPosition(close);
            }
            if(gamepad1.aWasPressed()) {
                spool.setPosition(unspooled);
                sleep(2000);
                openClose.setPosition(open);


            }

            if(gamepad1.xWasPressed()) {
                spin1.setPower(0);
                spin2.setPower(0);
                spin3.setPower(0);
            }
            telemetry.addData("rotationEncoders" , Rotation.getCurrentPosition());
            telemetry.update();
            }

        }

    }



