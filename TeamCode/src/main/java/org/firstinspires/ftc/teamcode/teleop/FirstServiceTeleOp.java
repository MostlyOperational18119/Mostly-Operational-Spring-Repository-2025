package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.services.Service.TeleOpService;
import org.firstinspires.ftc.teamcode.services.ServiceOpMode;

@TeleOp(name = "First Service TeleOp", group = "First Service")
public class FirstServiceTeleOp extends ServiceOpMode {
    @Override
    public boolean isTeleOp() {
        return true;
    }

    @Override
    public TeleOpService getService() {
        return new FirstServiceTeleOpService(driveServiceInputQueue, visionServiceOutputQueue, teleOpServiceInputQueue);
    }

}
