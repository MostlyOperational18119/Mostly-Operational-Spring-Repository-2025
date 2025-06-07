package org.firstinspires.ftc.teamcode.services.Service.Planner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class State {
    public DistanceUnit distanceUnit;
    public double x;
    public double y;
    public AngleUnit angleUnit;
    public double yaw;
    public double velocity;
    public double angularRate;

    public State(DistanceUnit distanceUnit, double x, double y, AngleUnit angleUnit, double yaw, double velocity, double angularRate) {
        this.distanceUnit = distanceUnit;
        this.x = x;
        this.y = y;
        this.angleUnit = angleUnit;
        this.yaw = yaw;
    }

    public State() {
        this.distanceUnit = DistanceUnit.CM;
        this.x = 0;
        this.y = 0;
        this.angleUnit = AngleUnit.RADIANS;
        this.yaw = 0;
    }

    public double getX(DistanceUnit distanceUnit) {
        return distanceUnit.fromUnit(this.distanceUnit, this.x);
    }

    public double getY(DistanceUnit distanceUnit) {
        return distanceUnit.fromUnit(this.distanceUnit, this.y);
    }

    public double getVelocity(DistanceUnit distanceUnit) {
        return distanceUnit.fromUnit(this.distanceUnit, this.velocity);
    }

    public double getYaw(AngleUnit angleUnit) {
        return angleUnit.fromUnit(this.angleUnit, this.yaw);
    }

    public double getAngularRate(AngleUnit angleUnit) {
        return angleUnit.fromUnit(this.angleUnit, this.angularRate);
    }
}
