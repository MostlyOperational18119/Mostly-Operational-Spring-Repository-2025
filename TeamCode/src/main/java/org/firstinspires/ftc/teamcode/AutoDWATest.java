package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.services.Service.TeleOpService;
import org.firstinspires.ftc.teamcode.services.ServiceOpMode;

@Autonomous(name="DWA Test Auto")
public class AutoDWATest extends ServiceOpMode {
    @Override
    public boolean isTeleOp() {
        return false;
    }

    @Override
    public TeleOpService getService() {
        return null;
    }
}
