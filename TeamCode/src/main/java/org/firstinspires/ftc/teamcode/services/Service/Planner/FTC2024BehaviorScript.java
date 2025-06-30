package org.firstinspires.ftc.teamcode.services.Service.Planner;

import org.apache.commons.math4.legacy.core.Pair;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Function;

public class FTC2024BehaviorScript implements BehaviorScript {
    private boolean hasScoringElement = false;

    @Override
    public void setHasScoringElement(boolean hasScoringElement) {
        this.hasScoringElement = hasScoringElement;
    }

    @Override
    public boolean getHasScoringElement() {
        return hasScoringElement;
    }

    @Override
    public Pair<Pose2D, Optional<Function<LinkedBlockingQueue<DriveServiceInput>, Void>>> getGoal(
            ArrayList<FieldNode> fieldNodes, double[] robotState, Alliance alliance
    ) {
        Pose2D targetPose = null;
        Optional<Function<LinkedBlockingQueue<DriveServiceInput>, Void>> func = Optional.empty();

        if (hasScoringElement) {
            // We shall score in the basket (for now)

            switch (alliance) {
                case RED:
                    targetPose = new Pose2D(DistanceUnit.METER, 0.4, 0.4, AngleUnit.DEGREES, -135);
                    break;
                case BLUE:
                    targetPose = new Pose2D(DistanceUnit.METER, 3.3528, 3.3528, AngleUnit.DEGREES, 90);
                    break;
                case NONE:
                    return new Pair<>(null, Optional.empty());
            }

            func = Optional.of(this::placeInBasket);


        } else {
            // We need to grab one :D
            for (FieldNode node : fieldNodes) {
                if (node.type == FieldNode.NodeType.SCORING_ELEMENT && node.alliance == alliance) {
                    targetPose = node.position;
                    break;
                }
            }

            if (targetPose == null) {
                // In the future, we will grab one from the submersible
                return new Pair<>(null, Optional.empty());
            }
        }

        return new Pair<>(targetPose, func);
    }

    Void placeInBasket(LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue) {
        // Nothing for simulator (for now)
        return null;
    }

    Void pickupSample(LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue) {
        // Nothing for simulator (for now)
        return null;
    }
}
