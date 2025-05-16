package org.firstinspires.ftc.teamcode.services.service;

import org.firstinspires.ftc.teamcode.services.Drive.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.communication.VisionServiceOutput;

import java.util.concurrent.LinkedBlockingQueue;


// Heavily inspired by https://github.com/amslabtech/dwa_planner
public class PlannerService implements Runnable {
    private LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue;
    private LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue;

    public PlannerService(LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue, LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue) {
        this.visionServiceOutputQueue = visionServiceOutputQueue;
        this.driveServiceInputQueue = driveServiceInputQueue;
    }

    @Override
    public void run() {

    }
}
