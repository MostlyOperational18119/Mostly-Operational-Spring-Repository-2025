package org.firstinspires.ftc.teamcode.services.Service.Planner;

import org.apache.commons.math4.legacy.core.Pair;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Function;

public interface BehaviorScript {
    void setHasScoringElement(boolean hasScoringElement);
    boolean getHasScoringElement();

    // Function is called upon completion
    Pair<Pose2D, Optional<Function<LinkedBlockingQueue<DriveServiceInput>, Void>>> getGoal(
            ArrayList<FieldNode> fieldNodes, double[] robotState, Alliance alliance
    );
}
