package org.firstinspires.ftc.teamcode.services.Service;

import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.concurrent.LinkedBlockingQueue;

public abstract class TeleOpService implements Runnable {
    LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue;
    LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue;

    TeleOpService(LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue, LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue) {
        this.driveServiceInputQueue = driveServiceInputQueue;
        this.visionServiceOutputQueue = visionServiceOutputQueue;
    }
}
