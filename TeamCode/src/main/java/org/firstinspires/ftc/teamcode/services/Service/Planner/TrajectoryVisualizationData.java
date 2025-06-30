
package org.firstinspires.ftc.teamcode.services.Service.Planner;

import java.util.List;

public class TrajectoryVisualizationData {
    public static class EvaluatedTrajectory {
        public final double[][] trajectory;
        public final double cost;
        public final boolean isSelected;

        public EvaluatedTrajectory(double[][] trajectory, double cost, boolean isSelected) {
            this.trajectory = trajectory;
            this.cost = cost;
            this.isSelected = isSelected;
        }
    }

    public final List<EvaluatedTrajectory> evaluatedTrajectories;
    public final double[] robotState;

    public TrajectoryVisualizationData(List<EvaluatedTrajectory> evaluatedTrajectories, double[] robotState) {
        this.evaluatedTrajectories = evaluatedTrajectories;
        this.robotState = robotState.clone();
    }
}