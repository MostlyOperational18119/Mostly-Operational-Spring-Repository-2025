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
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;


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

    private ArrayList<double[]> obstacles = new ArrayList<>();

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

                double toGoalCost = TO_GOAL_COST_GAIN * calcToGoalCost(trajectory, goal);
                double speedCost = SPEED_COST_GAIN * (MAX_LINEAR_VELOCITY - trajectory[trajectory.length - 1][3]);
                double obCost = OBSTACLE_COST_GAIN * calcObstacleCost(trajectory);
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

    double calcObstacleCost(double[][] trajectory) {
        double[] ox = getColumn(obstacles, 0);
        double[] oy = getColumn(obstacles, 1);

        double[][] dx = subtractArrays(getColumn(trajectory, 0),  ox);
        double[][] dy = subtractArrays(getColumn(trajectory, 1), oy);

        double[][] r = hypot2DArray(dx, dy);

        double[] yaw = getColumn(trajectory, 2);

        // For now, so compile works and I can push without breaking other peoples builds
        return 0.0;
    }

    double[] getColumn(double[][] src, int index) {
        return Arrays.stream(src).mapToDouble(doubles -> doubles[index]).toArray();
    }

    double[] getColumn(ArrayList<double[]> src, int index) {
        return src.stream().mapToDouble(doubles -> doubles[index]).toArray();
    }

    // Waow
    double[][] subtractArrays(double[] src1, double[] src2) {
        double[][] result = new double[src1.length][src2.length];

        for (int i = 0; i < src1.length; i++) {
            for (int j = 0; j < src2.length; j++) {
                result[i][j] = src1[i] - src2[j];
            }
        }

        return result;
    }

    double[][] hypot2DArray(double[][] x, double[][] y) {
        // Better not be empty, or it'll crash
        assert x.length <= 1;
        assert x[0].length <= 1;

        // Better be the same dimensions
        assert x.length == y.length;
        assert x[0].length == y[0].length;

        double[][] result = new double[x.length][x[0].length];

        for (int i = 0; i < x.length; i++) {
            for (int j = 0; j < x[0].length; j++) {
                // Set the result in the corresponding slot to the hypotenuse of the corresponding x and y :D
                result[i][j] = Math.hypot(x[i][j], y[i][j]);
            }
        }

        return result;
    }

    double[][][] transpose3DArray(double[][][] src, int[] axes) {
        int[] dims = new int[] { src.length, src[0].length, src[0][0].length };

        int newDim0 = dims[axes[0]];
        int newDim1 = dims[axes[1]];
        int newDim2 = dims[axes[2]];

        double[][][] transposedArray = new double[newDim0][newDim1][newDim2];

        for (int i = 0; i < dims[0]; i++) {
            for (int j = 0; j < dims[1]; j++) {
                for (int k = 0; k < dims[2]; k++) {
                    int[] indexes = new int[]{i, j, k};
                    transposedArray
                            [indexes[axes[0]]]
                            [indexes[axes[1]]]
                            [indexes[axes[2]]] = src[i][j][k];
                }
            }
        }

        return transposedArray;
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
