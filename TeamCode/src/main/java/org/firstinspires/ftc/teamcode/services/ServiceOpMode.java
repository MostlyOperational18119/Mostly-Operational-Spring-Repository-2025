package org.firstinspires.ftc.teamcode.services;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.services.Communication.HeartbeatInput;
import org.firstinspires.ftc.teamcode.services.Communication.HeartbeatOutput;
import org.firstinspires.ftc.teamcode.services.Communication.TeleOpServiceInput;
import org.firstinspires.ftc.teamcode.services.Service.DriveService;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Service.Planner.PlannerService;
import org.firstinspires.ftc.teamcode.services.Service.TeleOpService;
import org.firstinspires.ftc.teamcode.services.Service.VisionService;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.Objects;
import java.util.concurrent.LinkedBlockingQueue;

public abstract class ServiceOpMode extends OpMode {
    protected LinkedBlockingQueue<HeartbeatInput> visionServiceInputQueue;
    protected LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue;
    protected LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue;
    protected LinkedBlockingQueue<HeartbeatOutput> driveServiceOutputQueue;
    protected LinkedBlockingQueue<TeleOpServiceInput> teleOpServiceInputQueue;


    private DriveService driveService;
    private Thread driveServiceThread;
    private VisionService visionService;
    private Thread visionServiceThread;
    private PlannerService plannerService;
    private Thread plannerServiceThread;

    public abstract boolean isTeleOp();
    public abstract TeleOpService getService();
    private TeleOpService teleOpService;
    private Thread teleOpServiceThread;


    @Override
    public void init() {
        boolean teleOp = isTeleOp();

        this.visionServiceInputQueue = new LinkedBlockingQueue<>();
        this.visionServiceOutputQueue = new LinkedBlockingQueue<>();
        this.driveServiceInputQueue = new LinkedBlockingQueue<>();
        this.driveServiceOutputQueue = new LinkedBlockingQueue<>();
        this.teleOpServiceInputQueue = new LinkedBlockingQueue<>();

        this.driveService = new DriveService(hardwareMap, driveServiceInputQueue, true);
//        this.visionService = new VisionService(hardwareMap, visionServiceInputQueue, visionServiceOutputQueue);
        this.plannerService = new PlannerService(hardwareMap, visionServiceOutputQueue, driveServiceInputQueue);
        if (teleOp) {
            this.teleOpService = getService();
        }


        try {
            driveServiceThread = new Thread(driveService);
//            visionServiceThread = new Thread(visionService);
            plannerServiceThread = new Thread(plannerService);
            if (teleOp) {
                teleOpServiceThread = new Thread(teleOpService);
            }

            driveServiceThread.start();
//            visionServiceThread.start();
            plannerServiceThread.start();
            if (teleOp) {
                teleOpServiceThread.start();
            }
        } catch (Exception e) {
            telemetry.addData("Failed to start services", e.getLocalizedMessage());
            Log.e("ServiceOpMode", Objects.requireNonNull(e.getLocalizedMessage()));
            terminateOpModeNow();
        }
    }

    @Override
    public void loop() {
//        requestOpModeStop();

        if (isTeleOp()) {
            teleOpServiceInputQueue.add(new TeleOpServiceInput(gamepad1, gamepad2));
        }
    }

    @Override
    public void stop() {
        if (driveServiceThread != null) driveServiceThread.interrupt();
        if (visionServiceThread != null) visionServiceThread.interrupt();
        if (plannerServiceThread != null) plannerServiceThread.interrupt();
        if (isTeleOp()) teleOpServiceThread.interrupt();
    }
}
