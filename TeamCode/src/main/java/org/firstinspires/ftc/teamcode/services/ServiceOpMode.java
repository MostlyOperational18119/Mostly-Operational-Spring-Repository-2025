package org.firstinspires.ftc.teamcode.services;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.services.Communication.HeartbeatInput;
import org.firstinspires.ftc.teamcode.services.Communication.HeartbeatOutput;
import org.firstinspires.ftc.teamcode.services.Service.DriveService;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Service.PlannerService;
import org.firstinspires.ftc.teamcode.services.Service.VisionService;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.concurrent.LinkedBlockingQueue;

public class ServiceOpMode extends OpMode {
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


    @Override
    public void init() {
        this.visionServiceInputQueue = new LinkedBlockingQueue<>();
        this.visionServiceOutputQueue = new LinkedBlockingQueue<>();
        this.driveServiceInputQueue = new LinkedBlockingQueue<>();
        this.driveServiceOutputQueue = new LinkedBlockingQueue<>();

        this.driveService = new DriveService(hardwareMap, driveServiceInputQueue);
        this.visionService = new VisionService(hardwareMap, visionServiceInputQueue, visionServiceOutputQueue);
        this.plannerService = new PlannerService(visionServiceOutputQueue, driveServiceInputQueue);

        try {
            driveServiceThread = new Thread(driveService);
            visionServiceThread = new Thread(visionService);
            plannerServiceThread = new Thread(plannerService);

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

    }

    @Override
    public void stop() {
        driveServiceThread.interrupt();
        visionServiceThread.interrupt();
        plannerServiceThread.interrupt();
    }
}
