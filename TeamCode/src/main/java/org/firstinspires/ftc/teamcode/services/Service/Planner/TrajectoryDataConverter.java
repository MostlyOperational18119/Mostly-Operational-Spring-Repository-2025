package org.firstinspires.ftc.teamcode.services.Service.Planner;

import com.google.protobuf.InvalidProtocolBufferException;

import org.firstinspires.ftc.teamcode.proto.TrajectoryVisualizationDataProtobuf;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A utility class to handle serialization and deserialization between
 * the TrajectoryVisualizationData Java class and its Protobuf representation.
 */
public class TrajectoryDataConverter {

    /**
     * Serializes a TrajectoryVisualizationData Java object into its Protobuf message equivalent.
     *
     * @param javaData The source Java object.
     * @return The corresponding Protobuf message.
     */
    public static TrajectoryVisualizationDataProtobuf.TrajectoryVisualizationData serialize(TrajectoryVisualizationData javaData) {
        // ... (Serialization code from the previous answer remains the same)
        TrajectoryVisualizationDataProtobuf.TrajectoryVisualizationData.Builder dataBuilder =
                TrajectoryVisualizationDataProtobuf.TrajectoryVisualizationData.newBuilder();

        if (javaData.robotState != null) {
            List<Double> robotStateList = Arrays.stream(javaData.robotState)
                    .boxed()
                    .collect(Collectors.toList());
            dataBuilder.addAllRobotState(robotStateList);
        }

        if (javaData.evaluatedTrajectories != null) {
            for (TrajectoryVisualizationData.EvaluatedTrajectory javaEvalTrajectory : javaData.evaluatedTrajectories) {
                TrajectoryVisualizationDataProtobuf.EvaluatedTrajectory.Builder evalTrajectoryBuilder =
                        TrajectoryVisualizationDataProtobuf.EvaluatedTrajectory.newBuilder();

                evalTrajectoryBuilder.setCost(javaEvalTrajectory.cost);
                evalTrajectoryBuilder.setIsSelected(javaEvalTrajectory.isSelected);

                if (javaEvalTrajectory.trajectory != null) {
                    for (double[] javaPoint : javaEvalTrajectory.trajectory) {
                        TrajectoryVisualizationDataProtobuf.EvaluatedTrajectory.Trajectory.Builder pointBuilder =
                                TrajectoryVisualizationDataProtobuf.EvaluatedTrajectory.Trajectory.newBuilder();
                        if (javaPoint != null) {
                            List<Double> pointList = Arrays.stream(javaPoint).boxed().collect(Collectors.toList());
                            pointBuilder.addAllPoint(pointList);
                        }
                        evalTrajectoryBuilder.addTrajectory(pointBuilder.build());
                    }
                }
                dataBuilder.addEvaluatedTrajectories(evalTrajectoryBuilder.build());
            }
        }
        return dataBuilder.build();
    }

    /**
     * Deserializes a byte array into a TrajectoryVisualizationData Java object.
     *
     * @param data The byte array received from the network/file.
     * @return The deserialized TrajectoryVisualizationData Java object.
     * @throws InvalidProtocolBufferException if the data cannot be parsed.
     */
    public static TrajectoryVisualizationData deserialize(byte[] data) throws InvalidProtocolBufferException {
        // 1. Parse the byte array back into the Protobuf message object.
        TrajectoryVisualizationDataProtobuf.TrajectoryVisualizationData protoData =
                TrajectoryVisualizationDataProtobuf.TrajectoryVisualizationData.parseFrom(data);

        // 2. Convert the Protobuf robot_state back to a double[].
        double[] robotState = protoData.getRobotStateList().stream()
                .mapToDouble(Double::doubleValue)
                .toArray();

        // 3. Convert the Protobuf evaluated_trajectories back to a List.
        List<TrajectoryVisualizationData.EvaluatedTrajectory> evaluatedTrajectories =
                protoData.getEvaluatedTrajectoriesList().stream()
                        .map(protoEvalT -> {
                            // 3a. Convert the nested trajectory (repeated Trajectory -> double[][])
                            double[][] trajectory = protoEvalT.getTrajectoryList().stream()
                                    .map(protoPoint -> protoPoint.getPointList().stream()
                                            .mapToDouble(Double::doubleValue)
                                            .toArray())
                                    .toArray(double[][]::new);

                            // 3b. Get the simple fields.
                            double cost = protoEvalT.getCost();
                            boolean isSelected = protoEvalT.getIsSelected();

                            // 3c. Create the new Java object.
                            return new TrajectoryVisualizationData.EvaluatedTrajectory(trajectory, cost, isSelected);
                        })
                        .collect(Collectors.toList());

        // 4. Construct the final Java object and return it.
        return new TrajectoryVisualizationData(evaluatedTrajectories, robotState);
    }
}