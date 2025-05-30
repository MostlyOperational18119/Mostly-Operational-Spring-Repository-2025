package org.firstinspires.ftc.teamcode.services.Service;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;

import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

public class DriveService implements Runnable {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor verticalSlide;
    DcMotor horizontalSlide;
    HardwareMap hardwareMap;

    final private LinkedBlockingQueue<DriveServiceInput> inputQueue;

    public DriveService(HardwareMap hardwareMap, LinkedBlockingQueue<DriveServiceInput> inputQueue) {
        this.hardwareMap = hardwareMap;
        this.inputQueue = inputQueue;
    }

    @Override
    public void run() {
        try {
            while (true) {
                DriveServiceInput input = inputQueue.poll(5, TimeUnit.SECONDS);
                if (input.mode == DriveServiceInput.DriveServiceInputMode.MANUAL) {
                    motorFL.setPower(input.flSpeed);
                    motorFR.setPower(input.frSpeed);
                    motorBL.setPower(input.blSpeed);
                    motorBR.setPower(input.brSpeed);
                    verticalSlide.setPower(input.verticalSpeed);
                    horizontalSlide.setPower(input.horizontalSpeed);
                }
            }
        } catch (InterruptedException e) {

        }
    }

    public void setup() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        verticalSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
