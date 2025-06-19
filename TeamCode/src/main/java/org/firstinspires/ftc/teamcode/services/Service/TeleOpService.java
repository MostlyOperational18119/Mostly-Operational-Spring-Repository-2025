package org.firstinspires.ftc.teamcode.services.Service;

import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.TeleOpServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.concurrent.LinkedBlockingQueue;

public abstract class TeleOpService implements Runnable {
    protected LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue;
    protected LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue;
    protected LinkedBlockingQueue<TeleOpServiceInput> teleOpServiceInputQueue;

    public TeleOpService(LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue, LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue, LinkedBlockingQueue<TeleOpServiceInput> teleOpServiceInputQueue) {
        this.driveServiceInputQueue = driveServiceInputQueue;
        this.visionServiceOutputQueue = visionServiceOutputQueue;
        this.teleOpServiceInputQueue = teleOpServiceInputQueue;
    }
}
