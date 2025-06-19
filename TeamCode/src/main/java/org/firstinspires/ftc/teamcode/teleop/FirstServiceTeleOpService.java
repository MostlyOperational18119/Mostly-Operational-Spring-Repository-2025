package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.TeleOpServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;
import org.firstinspires.ftc.teamcode.services.Service.TeleOpService;

import java.util.concurrent.LinkedBlockingQueue;

public class FirstServiceTeleOpService extends TeleOpService {
    public FirstServiceTeleOpService(LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue, LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue, LinkedBlockingQueue<TeleOpServiceInput> teleOpServiceInputQueue) {
        super(driveServiceInputQueue, visionServiceOutputQueue, teleOpServiceInputQueue);
    }

    @Override
    public void run() {
        while (true) {
            TeleOpServiceInput input = teleOpServiceInputQueue.poll();

            if (input != null) {
                if (input.gamepad1.a) {
                    Log.d("FirstServiceTeleOpService", "A button pressed");
                }
            }
        }
    }
}
