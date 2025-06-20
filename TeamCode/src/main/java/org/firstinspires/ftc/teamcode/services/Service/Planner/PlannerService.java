package org.firstinspires.ftc.teamcode.services.Service.Planner;


import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.LinkedBlockingQueue;


// Heavily inspired by https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DynamicWindowApproach/dynamic_window_approach.py
// TODO: Ensure that I'm meeting the MIT license terms
public class PlannerService implements Runnable {
    private LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue;
    private LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue;
    private GoBildaPinpointDriver pinpoint;

    // DWA Variables

    final private double TO_GOAL_COST_GAIN = 0.15;
    final private double SPEED_COST_GAIN = 1.0;
    final private double OBSTACLE_COST_GAIN = 1.0;

    // DWA FUN FACTS
    // State [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]

    // Kinematics constants
    final private double MAX_LINEAR_VELOCITY = 0.5;
    final private double MAX_ANGULAR_VELOCITY = Math.toRadians(20.0);
    final private double MAX_LINEAR_ACCELERATION = 0.2;
    final private double MAX_ANGULAR_ACCELERATION = Math.toRadians(50.0);
    final private double VELOCITY_RESOLUTION = 0.01;
    final private double ANGULAR_VELOCITY_RESOLUTION = Math.toRadians(1.0);

    private ArrayList<Pose2D> obstacles = new ArrayList<>();

    // DWA Methods

    private double[] dynamicWindow(double[] state) {
        double[] Vs = new double[] {
                0.0, MAX_LINEAR_VELOCITY,
                -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY
        };
        double[] Vd = new double[] {
                state[3] - MAX_LINEAR_VELOCITY * 0.1,
                state[3] + MAX_LINEAR_VELOCITY * 0.1,
                state[4] - MAX_ANGULAR_VELOCITY * 0.1,
                state[4] + MAX_ANGULAR_VELOCITY * 0.1,
        };

        return new double[] {
                Math.max(Vs[0], Vd[0]), Math.min(Vs[1], Vd[1]),
                Math.max(Vs[2], Vd[2]), Math.min(Vs[3], Vd[3])
        };
    }

    private HardwareMap hardwareMap;

    Pair<double[], double[][]> calcControlAndTrajectory(double[] state, double[] dynWin, Pose2D goal) {
        double minimumCost = Double.POSITIVE_INFINITY;
        double[] bestU = new double[] { 0.0, 0.0 };
        double[][] bestTrajectory = new double[][] { state };

        for (double v = dynWin[0]; v <= dynWin[1]; v += VELOCITY_RESOLUTION) {
            for (double y = dynWin[2]; y <= dynWin[3]; y += ANGULAR_VELOCITY_RESOLUTION) {
                double[][] trajectory = predictTrajectory(state, v, y);

                double toGoalCost = calcToGoalCost(trajectory, goal);
                double speedCost = SPEED_COST_GAIN * (MAX_LINEAR_VELOCITY - trajectory[trajectory.length - 1][3]);
//                double obCost = ;
            }

        }

        return new Pair<>(bestU, bestTrajectory);
    }

    double calcToGoalCost(double[][] trajectory, Pose2D goal) {
        double dx = goal.getX(DistanceUnit.CM) - trajectory[trajectory.length - 1][0];
        double dy = goal.getY(DistanceUnit.CM) - trajectory[trajectory.length - 1][1];

        double errorAngle = Math.atan2(dy, dx);
        double costAngle = errorAngle - trajectory[trajectory.length - 1][2];

        // Le cost
        return Math.abs(Math.atan2(Math.sin(costAngle), Math.cos(costAngle)));
    }




    double[][] predictTrajectory(double[] state, double v, double y) {
        double[][] trajectory = new double[][] { state };

        double time = 0.0;

        while (time <= 3.0) {
            double[] x = motionModel(state, new double[] { v, y });

            trajectory = Arrays.copyOf(trajectory, trajectory.length + 1);
            trajectory[trajectory.length - 1] = x;

            time += 0.1;
        }

        return trajectory;
    }

    // This better not be incorrect or I will scream
    double[] motionModel(double[] state, double[] u) {
        state[2] += u[1] * 0.1;
        state[0] += u[0] * Math.cos(state[2]) * 0.1;
        state[1] += u[0] * Math.cos(state[2]) * 0.1;
        state[3] = u[0];
        state[4] = u[1];

        return state;
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

//        State robotState = new State();

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
