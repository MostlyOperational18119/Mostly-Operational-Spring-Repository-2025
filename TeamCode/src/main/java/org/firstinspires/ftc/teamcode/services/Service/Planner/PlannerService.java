package org.firstinspires.ftc.teamcode.services.Service.Planner;


import android.util.Log;
import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math4.legacy.linear.Array2DRowRealMatrix;
import org.apache.commons.math4.legacy.linear.ArrayRealVector;
import org.apache.commons.math4.legacy.linear.MatrixUtils;
import org.apache.commons.math4.legacy.linear.RealMatrix;
import org.apache.commons.math4.legacy.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.OptionalDouble;
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
    final private double ROBOT_STUCK_FLAG = 0.001;

    // DWA FUN FACTS
    // State is [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]

    // Kinematics constants
    final private double MAX_LINEAR_VELOCITY = 1.0;
    final private double MAX_ANGULAR_VELOCITY = Math.toRadians(40.0);
    final private double MAX_LINEAR_ACCELERATION = 0.2;
    final private double MAX_ANGULAR_ACCELERATION = Math.toRadians(40.0);
    final private double VELOCITY_RESOLUTION = 0.01;
    final private double ANGULAR_VELOCITY_RESOLUTION = Math.toRadians(0.1);
    final private double ROBOT_LENGTH = 0.16; // BOTH IN METERS
    final private double ROBOT_WIDTH = 0.16; // TODO: GET THE CORRECT MEASUREMENTS, OTHERWISE WE WILL CRASH INTO OBSTACLES AND WE WILL BE SAD

    public static ArrayList<double[]> obstacles = new ArrayList<>(Arrays.asList(
            new double[]{-1.0, -1.0},
            new double[]{0.0, 2.0},
            new double[]{4.0, 2.0},
            new double[]{5.0, 4.0},
            new double[]{5.0, 5.0},
            new double[]{5.0, 6.0},
            new double[]{8.0, 9.0},
            new double[]{7.0, 9.0},
            new double[]{8.0, 10.0},
            new double[]{9.0, 11.0},
            new double[]{12.0, 13.0},
            new double[]{12.0, 12.0},
            new double[]{15.0, 15.0},
            new double[]{13.0, 13.0}
    ));

    private Pose2D goal = new Pose2D(DistanceUnit.METER, 0.5, 0.5, AngleUnit.RADIANS, 0.0);
    private boolean goalActive = true;

    // DWA Methods

    private double[] dynamicWindow(double[] state) {
        double[] Vs = new double[]{
                0.0, MAX_LINEAR_VELOCITY,
                -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY
        };
        double[] Vd = new double[]{
                state[3] - MAX_LINEAR_ACCELERATION * 0.1,
                state[3] + MAX_LINEAR_ACCELERATION * 0.1,
                state[4] - MAX_ANGULAR_ACCELERATION * 0.1,
                state[4] + MAX_ANGULAR_ACCELERATION * 0.1,
        };

        return new double[]{
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

                double finalCost = toGoalCost + speedCost + obCost;

                if (minimumCost >= finalCost) {
                    minimumCost = finalCost;
                    bestU = new double[] { v, y };
                    bestTrajectory = trajectory;

                    if (Math.abs(v) < ROBOT_STUCK_FLAG && Math.abs(state[3]) < ROBOT_STUCK_FLAG) {
                        bestU[1] = -MAX_ANGULAR_ACCELERATION;
                    }
                }
            }

        }

        return new Pair<>(bestU, bestTrajectory);
    }

    double calcToGoalCost(double[][] trajectory, Pose2D goal) {
        double dx = goal.getX(DistanceUnit.METER) - trajectory[trajectory.length - 1][0];
        double dy = goal.getY(DistanceUnit.METER) - trajectory[trajectory.length - 1][1];

        double errorAngle = Math.atan2(dy, dx);
        double costAngle = errorAngle - trajectory[trajectory.length - 1][2];

        // Le cost
        return Math.abs(Math.atan2(Math.sin(costAngle), Math.cos(costAngle)));
    }

    double calcObstacleCost(double[][] trajectory) {
        double rMin = Double.POSITIVE_INFINITY;

        for (int i = 0; i < obstacles.size(); i++) {
            double ox = obstacles.get(i)[0];
            double oy = obstacles.get(i)[1];

            for (int j = 0; j < trajectory.length; j++) {
                double r = Math.hypot(
                        trajectory[j][0] - ox,
                        trajectory[j][1] - oy
                );

                if (rMin > r) {
                    rMin = r;
                }
            }
        }

        double halfRobotLength = ROBOT_LENGTH / 2.0;
        double halfRobotWidth = ROBOT_WIDTH / 2.0;

        for (int i = 0; i < trajectory.length; i++) {
            // Use epic matrix to check if we collided with the evil obstacles
            double currentYaw = trajectory[i][2];
            RealMatrix rotationMatrix = new Array2DRowRealMatrix(new double[][]{
                    {Math.cos(currentYaw), -Math.sin(currentYaw)},
                    {Math.sin(currentYaw), Math.cos(currentYaw)}
            });

            for (int j = 0; j < obstacles.size(); j++) {
                // Calculate the positon of the obstacle in relation to the robot
                double relativeX = obstacles.get(j)[0] - trajectory[i][0];
                double relativeY = obstacles.get(j)[1] - trajectory[i][1];

                // Make a vector for the relative positions
                RealVector relativeVector = new ArrayRealVector(new double[] {relativeX,relativeY});

                // Rotate our vector with our epic Matrix to put it in the robot's coordinate frame
                RealVector rotatedLocalObstacle = rotationMatrix.operate(relativeVector);

                // Get the values of the rotated vector
                double rotateRelativeObstacleX = rotatedLocalObstacle.getEntry(0);
                double rotateRelativeObstacleY = rotatedLocalObstacle.getEntry(1);

                // Did we collide?
                if (
                        rotateRelativeObstacleX <= halfRobotLength &&
                        rotateRelativeObstacleY <= halfRobotWidth &&
                        rotateRelativeObstacleX >= -halfRobotLength &&
                        rotateRelativeObstacleY >= -halfRobotWidth
                ) {
                    return Double.POSITIVE_INFINITY; // Uh oh stinky, we collided, so tell them this costs ten morbillion units
                }
            }
        }

        return 1.0 / rMin;
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
//        Log.i("PlannerService.predictTrajectory", Arrays.deepToString(trajectory));

        return trajectory;
    }

    // This was incorrect, I screamed
    double[] motionModel(double[] state, double[] u) {
        double[] newState = state.clone();

        newState[2] += u[1] * 0.1;
        newState[0] += u[0] * Math.cos(state[2]) * 0.1;
        newState[1] += u[0] * Math.sin(state[2]) * 0.1;
        newState[3] = u[0];
        newState[4] = u[1];

        return newState;
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
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            VisionServiceOutput visionOutput = visionServiceOutputQueue.poll();

            if (visionOutput != null) {
                if (visionOutput.type == VisionServiceOutput.VisionResultType.ROBOT) {
                    // Probably don't wanna crash into another bot, inform the planner of its presence and position
                    // Not implemented yet, so pretend it doesn't exist
                } else {
                    // Figure it out later :D
                }
            }

            if (!goalActive) continue;

            double[] state = new double[] {
                    pinpoint.getPosX(DistanceUnit.METER), pinpoint.getPosY(DistanceUnit.METER), pinpoint.getHeading(AngleUnit.RADIANS),
                    Math.hypot(pinpoint.getVelX(DistanceUnit.METER), pinpoint.getPosY(DistanceUnit.METER)), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            };

            double[] dynamicWindow = dynamicWindow(state);
            Pair<double[], double[][]> controlAndTrajectory = calcControlAndTrajectory(state, dynamicWindow, goal);
            double[] control = controlAndTrajectory.first;

            driveServiceInputQueue.add(new DriveServiceInput(control));

        }
    }
}
