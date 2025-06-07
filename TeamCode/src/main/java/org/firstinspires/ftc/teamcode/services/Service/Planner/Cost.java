package org.firstinspires.ftc.teamcode.services.Service.Planner;

public class Cost {
    double obstacleCost;
    double toGoalCost;
    double speedCost;
    double pathCost;
    double totalCost;

    Cost(double obstacleCost, double toGoalCost, double speedCost, double pathCost, double totalCost) {
        this.obstacleCost = obstacleCost;
        this.toGoalCost = toGoalCost;
        this.speedCost = speedCost;
        this.pathCost = pathCost;
        this.totalCost = totalCost;
    }

    void calculateTotalCost() {

    }
}
