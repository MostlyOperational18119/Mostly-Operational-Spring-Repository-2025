package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Picklebot Teleop")
public class PicklebotTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //SETTINGS//
        double flywheelGradient = 0.0015;
        double flywheelDisengagedPower = 0.1;
        double flywheelPower = 1.0;
        double DriveSpeedDivider = 5.0;
        int rotationMotorMax = 0; //unknown
        int rotationMotorMin = -300; //unknown
        double lockServoLocked = 0.55; //unknown
        double lockServoUnlocked = 0.2; //unknown
        double dropperServoIntake = 0.165; //unknown
        double dropperServoDrop = 0.06; //unknown
        double plungerServoLoaded = 0.04; //unknown
        double plungerServoFire = 0.23; //unknown
        //END SETTINGS//

        float driverLeftX = gamepad1.left_stick_x;
        float driverLeftY = -gamepad1.left_stick_y;
        float driverRightX = gamepad1.right_stick_x;
        int targetPosition = 0;

        Servo lockServo = hardwareMap.servo.get("lockServo");
        Servo plungerServo = hardwareMap.servo.get("plungerServo");
        Servo dropperServo = hardwareMap.servo.get("dropperServo");
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");
        DcMotor rotationMotor = hardwareMap.dcMotor.get("rotationMotor");
        DcMotor flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        DcMotor flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        DcMotor flywheelBottom = hardwareMap.dcMotor.get("flywheelBottom");

        flywheelBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setTargetPosition(0);
        rotationMotor.setPower(0.5);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dropperServo.setPosition(dropperServoIntake);

        boolean Flywheels = false;
        double flywheelCurrentPower = 0.1;
        Gamepad CurrentGamepad = new Gamepad();
        Gamepad PreviousGamepad = new Gamepad();
        waitForStart();
        while (opModeIsActive()) {
            PreviousGamepad.copy(CurrentGamepad);
            CurrentGamepad.copy(gamepad1);
            driverLeftX = gamepad1.left_stick_x;
            driverLeftY = -gamepad1.left_stick_y;
            driverRightX = gamepad1.right_stick_x;

            if (gamepad1.a && !PreviousGamepad.a) {
                Flywheels = !Flywheels;
            }

            if (Flywheels) {
                if (flywheelCurrentPower <= flywheelPower) {
                    flywheelCurrentPower += flywheelGradient;
                } else {
                    telemetry.addLine("max speed");
                }
            } else {
                if (flywheelCurrentPower >= flywheelDisengagedPower) {
                    flywheelCurrentPower -= flywheelGradient;
                }
            }

            if (gamepad1.dpad_up && !PreviousGamepad.dpad_up && targetPosition < rotationMotorMax) {
                targetPosition += 50;
            }
            if (gamepad1.dpad_down &&!PreviousGamepad.dpad_down && targetPosition > rotationMotorMin) {
                targetPosition -= 50;
            }

            if (gamepad1.b && !PreviousGamepad.b) {
                plungerServo.setPosition(plungerServoLoaded);
                sleep(2500);
                lockServo.setPosition(lockServoLocked);
            }

            if (gamepad1.x && !PreviousGamepad.x) {
                plungerServo.setPosition(plungerServoFire);
                sleep(2500);
                lockServo.setPosition(lockServoUnlocked);
            }

            if (gamepad1.y && !PreviousGamepad.y) {
                dropperServo.setPosition(dropperServoDrop);
                sleep(1000);
                dropperServo.setPosition(dropperServoIntake);
            }

            if (gamepad1.left_bumper && !PreviousGamepad.left_bumper) {
                flywheelPower += 0.05;
            }

            if (gamepad1.right_bumper && !PreviousGamepad.right_bumper) {
                flywheelPower -= 0.05;
            }

            motorFL.setPower((driverLeftY + driverLeftX + driverRightX) / DriveSpeedDivider);
            motorFR.setPower((driverLeftY - driverLeftX - driverRightX) / DriveSpeedDivider);
            motorBL.setPower((driverLeftY - driverLeftX + driverRightX) / DriveSpeedDivider);
            motorBR.setPower((driverLeftY + driverLeftX - driverRightX) / DriveSpeedDivider);

            flywheelLeft.setPower(flywheelCurrentPower);
            flywheelRight.setPower(flywheelCurrentPower);
            flywheelBottom.setPower(flywheelCurrentPower);

            rotationMotor.setTargetPosition(targetPosition);
            telemetry.addData("flywheels:", Flywheels);
            telemetry.addData("max power", flywheelPower);
            telemetry.addData("flywheelPower", flywheelCurrentPower);
            telemetry.addLine("You have to swap to flywheels disengaged then re engage them to update power settings");
            telemetry.update();
        }
    }
}
