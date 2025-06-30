package org.firstinspires.ftc.teamcode.services.Service.Planner;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math4.legacy.core.Pair;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Function;

public class PlannerService implements Runnable {
    private LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue;
    private LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue;

    // DWA Constants
    final private double DISTANCE_COST_GAIN = 1.0;
    final private double HEADING_COST_GAIN = 0.1;
    final private double ORIENTATION_COST_GAIN = 0.2;
    final private double SPEED_COST_GAIN = 0.5;
    final private double OBSTACLE_COST_GAIN = 1.0;
    final private double ROBOT_STUCK_FLAG = 0.01;
    final private double GOAL_TOLERANCE = 0.1;

    // Kinematics and Simulation Constants
    final private double MAX_LINEAR_VELOCITY = 1.0; // m/s
    final private double MAX_ANGULAR_VELOCITY = Math.toRadians(40.0); // rad/s
    final private double MAX_LINEAR_ACCELERATION = 0.2; // m/s²
    final private double MAX_ANGULAR_ACCELERATION = Math.toRadians(40.0); // rad/s²
    // FIX: Resolution must be smaller than acceleration * DT to sample the dynamic window properly.
    final private double LINEAR_VELOCITY_RESOLUTION = 0.005; // m/s
    final private double STRAFE_VELOCITY_RESOLUTION = 0.005; // m/s
    final private double ANGULAR_VELOCITY_RESOLUTION = Math.toRadians(0.5); // rad/s
    final private double ROBOT_LENGTH = 0.16; // meters
    final private double ROBOT_WIDTH = 0.16; // meters
    // FIX: Use a consistent time step for all physics calculations and the main loop.
    final private double DT = 0.1; // seconds

    // State: [x(m), y(m), yaw(rad), vx_robot(m/s), vy_robot(m/s), omega(rad/s)]
    final private FieldConfig fieldConfig;
    private ArrayList<FieldNode> fieldNodes = new ArrayList<>();
    private Alliance alliance;

    public Pose2D goal = new Pose2D(DistanceUnit.METER, 3.0, 3.0, AngleUnit.RADIANS, 0.0);
    private boolean goalActive = true;
    final private BehaviorScript behaviorScript;

    private GoBildaPinpointDriver pinpoint;

    private boolean isGoalReached(double[] state) {
        double goalX = goal.getX(DistanceUnit.METER);
        double goalY = goal.getY(DistanceUnit.METER);
        double distanceToGoal = Math.hypot(state[0] - goalX, state[1] - goalY);
        return distanceToGoal <= GOAL_TOLERANCE;
    }

    private double[] dynamicWindow(double[] state) {
        double[] Vs = new double[]{
                -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY,
                -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY,
                -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY
        };
        double[] Vd = new double[]{
                state[3] - MAX_LINEAR_ACCELERATION * DT,
                state[3] + MAX_LINEAR_ACCELERATION * DT,
                state[4] - MAX_LINEAR_ACCELERATION * DT,
                state[4] + MAX_LINEAR_ACCELERATION * DT,
                state[5] - MAX_ANGULAR_ACCELERATION * DT,
                state[5] + MAX_ANGULAR_ACCELERATION * DT,
        };

        return new double[]{
                Math.max(Vs[0], Vd[0]), Math.min(Vs[1], Vd[1]),
                Math.max(Vs[2], Vd[2]), Math.min(Vs[3], Vd[3]),
                Math.max(Vs[4], Vd[4]), Math.min(Vs[5], Vd[5])
        };
    }

    Pair<Pair<double[], double[][]>, TrajectoryVisualizationData> calcControlAndTrajectoryWithVisualization(double[] state, double[] dynWin, Pose2D goal) {
        double minimumCost = Double.POSITIVE_INFINITY;
        double[] bestU = new double[]{0.0, 0.0, 0.0};
        double[][] bestTrajectory = new double[][]{state};
        List<TrajectoryVisualizationData.EvaluatedTrajectory> allTrajectories = new ArrayList<>();

        for (double vx = dynWin[0]; vx <= dynWin[1]; vx += LINEAR_VELOCITY_RESOLUTION) {
            for (double vy = dynWin[2]; vy <= dynWin[3]; vy += STRAFE_VELOCITY_RESOLUTION) {
                if (Math.hypot(vx, vy) > MAX_LINEAR_VELOCITY) {
                    continue;
                }
                for (double omega = dynWin[4]; omega <= dynWin[5]; omega += ANGULAR_VELOCITY_RESOLUTION) {
                    double[] u = {vx, vy, omega};
                    double[][] trajectory = predictTrajectory(state, u);
                    double[] finalState = trajectory[trajectory.length - 1];

                    double distCost = DISTANCE_COST_GAIN * calcDistanceCost(finalState, goal);
                    double headingCost = HEADING_COST_GAIN * calcHeadingCost(finalState, goal);
                    double orientationCost = ORIENTATION_COST_GAIN * Math.abs(normalizeAngle(finalState[2] - goal.getHeading(AngleUnit.RADIANS)));
                    double speedCost = SPEED_COST_GAIN * (MAX_LINEAR_VELOCITY - Math.hypot(vx, vy));
                    double obCost = OBSTACLE_COST_GAIN * calcObstacleCost(trajectory);

                    double finalCost = distCost + headingCost + orientationCost + speedCost + obCost;
                    boolean isSelected = false;

                    if (minimumCost > finalCost) { // Use > instead of >= to prefer earlier, lower-velocity solutions in case of cost ties
                        minimumCost = finalCost;
                        bestU = u;
                        bestTrajectory = trajectory;
                        isSelected = true;

                        if (Math.hypot(vx, vy) < ROBOT_STUCK_FLAG && Math.hypot(state[3], state[4]) < ROBOT_STUCK_FLAG) {
                            bestU[2] = -MAX_ANGULAR_VELOCITY;
                        }
                    }

                    if (allTrajectories.size() < 200 || isSelected) {
                        allTrajectories.add(new TrajectoryVisualizationData.EvaluatedTrajectory(trajectory, finalCost, isSelected));
                    }
                }
            }
        }

        if (!allTrajectories.isEmpty()) {
            for (int i = 0; i < allTrajectories.size(); i++) {
                if (Arrays.deepEquals(allTrajectories.get(i).trajectory, bestTrajectory)) {
                    allTrajectories.set(i, new TrajectoryVisualizationData.EvaluatedTrajectory(allTrajectories.get(i).trajectory, allTrajectories.get(i).cost, true));
                    break;
                }
            }
        }

        TrajectoryVisualizationData visualizationData = new TrajectoryVisualizationData(allTrajectories, state);
        return new Pair<>(new Pair<>(bestU, bestTrajectory), visualizationData);
    }

    Pair<double[], double[][]> calcControlAndTrajectory(double[] state, double[] dynWin, Pose2D goal) {
        double minimumCost = Double.POSITIVE_INFINITY;
        double[] bestU = new double[]{0.0, 0.0, 0.0};
        double[][] bestTrajectory = new double[][]{state};

        for (double vx = dynWin[0]; vx <= dynWin[1]; vx += LINEAR_VELOCITY_RESOLUTION) {
            for (double vy = dynWin[2]; vy <= dynWin[3]; vy += STRAFE_VELOCITY_RESOLUTION) {
                if (Math.hypot(vx, vy) > MAX_LINEAR_VELOCITY) {
                    continue;
                }
                for (double omega = dynWin[4]; omega <= dynWin[5]; omega += ANGULAR_VELOCITY_RESOLUTION) {
                    double[] u = {vx, vy, omega};
                    double[][] trajectory = predictTrajectory(state, u);
                    double[] finalState = trajectory[trajectory.length - 1];

                    double distCost = DISTANCE_COST_GAIN * calcDistanceCost(finalState, goal);
                    double headingCost = HEADING_COST_GAIN * calcHeadingCost(finalState, goal);
                    double orientationCost = ORIENTATION_COST_GAIN * Math.abs(normalizeAngle(finalState[2] - goal.getHeading(AngleUnit.RADIANS)));
                    double speedCost = SPEED_COST_GAIN * (MAX_LINEAR_VELOCITY - Math.hypot(vx, vy));
                    double obCost = OBSTACLE_COST_GAIN * calcObstacleCost(trajectory);

                    double finalCost = distCost + headingCost + orientationCost + speedCost + obCost;
                    boolean isSelected = false;

                    if (minimumCost > finalCost) { // Use > instead of >= to prefer earlier, lower-velocity solutions in case of cost ties
                        minimumCost = finalCost;
                        bestU = u;
                        bestTrajectory = trajectory;
                        isSelected = true;

                        if (Math.hypot(vx, vy) < ROBOT_STUCK_FLAG && Math.hypot(state[3], state[4]) < ROBOT_STUCK_FLAG) {
                            bestU[2] = -MAX_ANGULAR_VELOCITY;
                        }
                    }
                }
            }
        }

        return new Pair<>(bestU, bestTrajectory);
    }

    double calcDistanceCost(double[] finalState, Pose2D goal) {
        return Math.hypot(finalState[0] - goal.getX(DistanceUnit.METER), finalState[1] - goal.getY(DistanceUnit.METER));
    }

    double calcHeadingCost(double[] finalState, Pose2D goal) {
        double dx = goal.getX(DistanceUnit.METER) - finalState[0];
        double dy = goal.getY(DistanceUnit.METER) - finalState[1];
        double angleToGoal = Math.atan2(dy, dx);
        double costAngle = angleToGoal - finalState[2];
        return Math.abs(normalizeAngle(costAngle));
    }

    double calcObstacleCost(double[][] trajectory) {
        double minDistance = Double.POSITIVE_INFINITY;
        final double safetyMargin = 0.5;

        for (double[] point : trajectory) {
            for (FieldNode fieldNode : fieldNodes) {
                if (fieldNode.type != FieldNode.NodeType.OBSTACLE) continue;
                double dx = Math.abs(point[0] - fieldNode.position.getX(DistanceUnit.METER)) - (fieldNode.width / 2.0);
                double dy = Math.abs(point[1] - fieldNode.position.getY(DistanceUnit.METER)) - (fieldNode.height / 2.0);
                double distanceToEdge = Math.hypot(Math.max(0, dx), Math.max(0, dy));
                minDistance = Math.min(minDistance, distanceToEdge);
            }
        }

        if (minDistance > safetyMargin) return 0.0;
        double clearance = minDistance - (ROBOT_WIDTH / 2.0);
        if (clearance <= 0.001) return Double.POSITIVE_INFINITY;
        return 1.0 / clearance;
    }

    double[][] predictTrajectory(double[] state, double[] u) {
        double[][] trajectory = new double[][]{state};
        double time = 0.0;
        double predictTime = 3.0;

        while (time <= predictTime) {
            double[] x = motionModel(trajectory[trajectory.length - 1], u, DT);
            trajectory = Arrays.copyOf(trajectory, trajectory.length + 1);
            trajectory[trajectory.length - 1] = x;
            time += DT;
        }
        return trajectory;
    }

    double[] motionModel(double[] state, double[] u, double dt) {
        double[] newState = new double[6];
        double yaw = state[2];
        double vx_robot = u[0];
        double vy_robot = u[1];

        double vx_world = vx_robot * Math.cos(yaw) - vy_robot * Math.sin(yaw);
        double vy_world = vx_robot * Math.sin(yaw) + vy_robot * Math.cos(yaw);

        newState[0] = state[0] + vx_world * dt;
        newState[1] = state[1] + vy_world * dt;
        newState[2] = state[2] + u[2] * dt;
        newState[3] = u[0];
        newState[4] = u[1];
        newState[5] = u[2];

        return newState;
    }

    double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2.0 * Math.PI;
        while (angleRad <= -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }

    public PlannerService(HardwareMap hardwareMap, LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue, LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue, FieldConfig fieldConfig, BehaviorScript behaviorScript, Alliance alliance) {
        // ... (constructor is unchanged)
        this.visionServiceOutputQueue = visionServiceOutputQueue;
        this.driveServiceInputQueue = driveServiceInputQueue;
        this.fieldConfig = fieldConfig;
        this.behaviorScript = behaviorScript;
        this.fieldNodes = new ArrayList<>(Arrays.asList(fieldConfig.getFixedFieldNodes()));
        this.fieldNodes.addAll(Arrays.asList(fieldConfig.getStartingDynamicFieldNodes()));
        this.alliance = alliance;
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Setup pinpoint
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(70, -180, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        double[] state;
        this.behaviorScript.setHasScoringElement(true);
        Log.i("PlannerService", String.format("Starting planner with goal at (%.3f, %.3f) meters%n", goal.getX(DistanceUnit.METER), goal.getY(DistanceUnit.METER)));

        while (true) {
            pinpoint.update();

            try {
                Thread.sleep((long) (DT * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }

            // GoBilda is fat, ugly, and stupid so their "Y-axis" is our X-axis and vice versa
            state = new double[] {
                    -pinpoint.getPosY(DistanceUnit.METER), pinpoint.getPosX(DistanceUnit.METER), pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS),
                    pinpoint.getVelY(DistanceUnit.METER), pinpoint.getVelX(DistanceUnit.METER), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            };

            packet.put("x", state[0]);
            packet.put("y", state[1]);
            packet.put("yaw", state[2]);
            packet.put("vx", state[3]);
            packet.put("vy", state[4]);
            packet.put("omega", state[5]);

            dashboard.sendTelemetryPacket(packet);

//            Log.i("PlannerService", String.format("New state: %s", Arrays.toString(state)));

            Log.i("PlannerService", "Angle: " + pinpoint.getHeading(AngleUnit.RADIANS));
            // ... (vision processing is unchanged) ...

            if (!goalActive) {
                Pair<Pose2D, Optional<Function<LinkedBlockingQueue<DriveServiceInput>, Void>>> behaviorGoalPair =
                        behaviorScript.getGoal(fieldNodes, state, alliance);
                goal = behaviorGoalPair.getKey();
                goalActive = true;
            }

            if (isGoalReached(state)) {
                Log.i("PlannerService", String.format("Goal reached! Robot at (%.3f, %.3f), goal at (%.3f, %.3f)%n",
                        state[0], state[1], goal.getX(DistanceUnit.METER), goal.getY(DistanceUnit.METER)));
                driveServiceInputQueue.add(new DriveServiceInput(new double[]{0.0, 0.0, 0.0}));
                goalActive = false;
                Log.i("PlannerService", "Goal deactivated, waiting for new goal.");
                continue; // Skip planning until a new goal is set
            }

            state[2] = normalizeAngle(state[2]);
            double[] dynamicWindow = dynamicWindow(state);
            Pair<double[], double[][]> result = calcControlAndTrajectory(state, dynamicWindow, goal);
            double[] control = result.getKey();

            driveServiceInputQueue.add(new DriveServiceInput(control));
        }
    }
}