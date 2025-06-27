package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

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
        double speedDiv = 2.0;

        int rotationTarget = 0;

        DcMotor Rotation = hardwareMap.get(DcMotor.class, "Rotation");

        Rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotation.setTargetPosition(0);
        Rotation.setPower(0.5);
        Rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //settings
        int rotationMax = 0;
        int rotationMin = -100;

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

                    if (gamepad1.dpad_up && rotationTarget < rotationMax) {
                        rotationTarget += 50;
                    }
                    if (gamepad1.dpad_down && rotationTarget > rotationMin) {
                        rotationTarget -= 50;

                    }




        }

    }


}
