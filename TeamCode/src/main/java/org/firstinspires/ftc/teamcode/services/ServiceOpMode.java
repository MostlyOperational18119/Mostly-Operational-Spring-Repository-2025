package org.firstinspires.ftc.teamcode.services;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.services.Communication.HeartbeatInput;
import org.firstinspires.ftc.teamcode.services.Communication.HeartbeatOutput;
import org.firstinspires.ftc.teamcode.services.Service.DriveService;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Service.Planner.PlannerService;
import org.firstinspires.ftc.teamcode.services.Service.TeleOpService;
import org.firstinspires.ftc.teamcode.services.Service.VisionService;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.concurrent.LinkedBlockingQueue;

public abstract class ServiceOpMode extends OpMode {
    private LinkedBlockingQueue<HeartbeatInput> visionServiceInputQueue;
    private LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue;
    private LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue;
    private LinkedBlockingQueue<HeartbeatOutput> driveServiceOutputQueue;


    private DriveService driveService;
    private Thread driveServiceThread;
    private VisionService visionService;
    private Thread visionServiceThread;
    private PlannerService plannerService;
    private Thread plannerServiceThread;

    abstract boolean isTeleOp();
    abstract TeleOpService getService();
    private TeleOpService service;
    private Thread serviceThread;


    @Override
    public void init() {
        Boolean teleop = isTeleOp();

        this.visionServiceInputQueue = new LinkedBlockingQueue<>();
        this.visionServiceOutputQueue = new LinkedBlockingQueue<>();
        this.driveServiceInputQueue = new LinkedBlockingQueue<>();
        this.driveServiceOutputQueue = new LinkedBlockingQueue<>();

        this.driveService = new DriveService(hardwareMap, driveServiceInputQueue);
        this.visionService = new VisionService(hardwareMap, visionServiceInputQueue, visionServiceOutputQueue);
        this.plannerService = new PlannerService(hardwareMap, visionServiceOutputQueue, driveServiceInputQueue);


        try {
            driveServiceThread = new Thread(driveService);
            visionServiceThread = new Thread(visionService);
            plannerServiceThread = new Thread(plannerService);
            if (teleop) {

            }

            driveServiceThread.start();
            visionServiceThread.start();
            plannerServiceThread.start();
        } catch (Exception e) {
            telemetry.addData("Failed to start services", e.getLocalizedMessage());
            terminateOpModeNow();
        }
    }

    @Override
    public void loop() {
        requestOpModeStop();
    }

    @Override
    public void stop() {
        driveServiceThread.interrupt();
        visionServiceThread.interrupt();
        plannerServiceThread.interrupt();
    }
}
