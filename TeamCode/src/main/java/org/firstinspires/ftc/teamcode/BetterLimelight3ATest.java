package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

public class BetterLimelight3ATest extends LinearOpMode {
    @Override
    public void runOpMode() {
        BetterLimelight3A limelight;
        Thread limelightThread;

        try {
            limelight = new BetterLimelight3A();
            limelightThread = new Thread(limelight);
        } catch (Exception e) {
            return;
        }

        limelightThread.start();


        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
