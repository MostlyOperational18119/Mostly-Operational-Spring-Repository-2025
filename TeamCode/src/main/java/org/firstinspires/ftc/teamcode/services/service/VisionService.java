package org.firstinspires.ftc.teamcode.services.service;


import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.services.HeartbeatInput;
import org.firstinspires.ftc.teamcode.services.communication.VisionServiceOutput;

import java.util.concurrent.LinkedBlockingQueue;

public class VisionService implements Runnable {
    final private LinkedBlockingQueue<HeartbeatInput> inputQueue;
    final private LinkedBlockingQueue<VisionServiceOutput> outputQueue;
    final private Limelight3A limelight;

    public VisionService(HardwareMap hardwareMap, LinkedBlockingQueue<HeartbeatInput> inputQueue, LinkedBlockingQueue<VisionServiceOutput> outputQueue) {
        this.inputQueue = inputQueue;
        this.outputQueue = outputQueue;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void run() {
        limelight.start();

        try {
            while (true) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    double[] pythonOutput = result.getPythonOutput();

                    if (pythonOutput[4] == -1 && pythonOutput[9] == -1) {
                        VisionServiceOutput output = getVisionServiceOutput(pythonOutput);

                        if (!outputQueue.offer(output)) {
                            // If output queue is full, clear it and put the new output in the newly emptied queue
                            outputQueue.clear();
                            outputQueue.put(output);
                        }
                    }
                }

                Thread.sleep(10);
            }
        } catch (InterruptedException e) {
            // Just exit the thread, if we were interrupted we are supposed to exit
        }
    }

    @NonNull
    private static VisionServiceOutput getVisionServiceOutput(double[] pythonOutput) {
        Position positionFromRobot = new Position(DistanceUnit.INCH, pythonOutput[5], pythonOutput[6], pythonOutput[7], -1);
        YawPitchRollAngles orientation = new YawPitchRollAngles(AngleUnit.DEGREES, (float) pythonOutput[10], (float) pythonOutput[11], (float) pythonOutput[12], -1);
        Pose3D pose3DFromRobot = new Pose3D(positionFromRobot, orientation);

        Pose2D pose2DFromRobot = new Pose2D(DistanceUnit.INCH, pythonOutput[5], pythonOutput[6], AngleUnit.DEGREES, pythonOutput[12]);

        VisionServiceOutput output = new VisionServiceOutput(pose2DFromRobot, pose3DFromRobot, VisionServiceOutput.VisionResultType.SCORING_ELEMENT);
        return output;
    }
}

