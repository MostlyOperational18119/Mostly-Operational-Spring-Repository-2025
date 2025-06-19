package org.firstinspires.ftc.teamcode.services.Service.Planner;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.LinkedBlockingQueue;


// Heavily inspired by https://github.com/turhancan97/Navigation-of-a-Unicycle-Like-Vehicle
// TODO: Ensure that I'm meeting the MIT license terms
public class PlannerService implements Runnable {
    private LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue;
    private LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue;
    private GoBildaPinpointDriver pinpoint;

    // DWA Variables

    // Kinematics constants
    final double MAX_LINEAR_VELOCITY = 0.5;
    final double MAX_ANGULAR_VELOCITY = Math.toRadians(20.0);
    final double MAX_LINEAR_ACCELERATION = 0.2;
    final double MAX_ANGULAR_ACCELERATION = Math.toRadians(50.0);
    final double VELOCITY_RESOLUTION = 0.01;
    final double ANGULAR_VELOCITY_RESOLUTION = Math.toRadians(1.0);

    // DWA Methods

    HardwareMap hardwareMap;

    State dwa(State currentState, Pose2D goal, ArrayList<Pose2D> obstacles, double obstacleRadius) {

        // For now...
        return currentState;
    }

    Window dynamicWindow(State currentState) {
        // TODO: Implement dt
        double vmin = Math.max(0, (currentState.velocity - MAX_LINEAR_ACCELERATION));
        double vmax = Math.min(MAX_LINEAR_VELOCITY, (currentState.velocity + MAX_LINEAR_ACCELERATION));
        double omin = Math.max(-MAX_ANGULAR_VELOCITY, (currentState.angularRate - MAX_ANGULAR_ACCELERATION));
        double omax = Math.min(MAX_ANGULAR_VELOCITY, (currentState.angularRate + MAX_ANGULAR_ACCELERATION));

        return new Window(vmin, vmax, omin, omax);
    }

    double[][] evaluate(State currentState, Window dynamicWindow, Pose2D goal, ArrayList<Pose2D> obstacles, double obstacleRadius) {
        for (double i = dynamicWindow.vmin; i <= dynamicWindow.vmax; i += VELOCITY_RESOLUTION) {
            for (double j = dynamicWindow.omin; j <= dynamicWindow.omax; j += ANGULAR_VELOCITY_RESOLUTION) {

            }
        }

        return null;
    }


    SimpleMatrix motionModel(State currentState, SimpleMatrix u) {
        SimpleMatrix F = new SimpleMatrix(
                new double[][] {
                        {1, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0},
                        {0, 0, 1, 0, 0},
                        {0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0}
                }
        );

        SimpleMatrix x = new SimpleMatrix(
                new double[]{
                        currentState.x,
                        currentState.y,
                        currentState.yaw,
                        currentState.velocity,
                        currentState.angularRate
                }
        );

        // TODO: ADD DT
        SimpleMatrix B = new SimpleMatrix(
                new double[][] {
                        {Math.cos(currentState.yaw), 0},
                        {Math.sin(currentState.yaw), 0},
                        {0, 0},
                        {1, 0},
                        {0, 1}
                }
        );

        return (F.mult(x)).plus(B.mult(u));
    }


    public PlannerService(HardwareMap hardwareMap, LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue, LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue) {
        this.visionServiceOutputQueue = visionServiceOutputQueue;
        this.driveServiceInputQueue = driveServiceInputQueue;
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void run() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // PLACEHOLDERS, UPDATE LATER
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        State robotState = new State();

        while (true) {
            VisionServiceOutput visionOutput = visionServiceOutputQueue.poll();

            if (visionOutput != null) {
                if (visionOutput.type == VisionServiceOutput.VisionResultType.ROBOT) {
                    // Probably don't wanna crash into another bot, inform the planner of its presence and position

                }
            }
        }
    }
}
