package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Autonomous(name = "Test PnP")
public class TestPnP extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        double[] tvec = new double[3];

        limelight.pipelineSwitch(0);

        sleep(50);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();

            telemetry.addData("Is connected", limelight.isConnected());
            telemetry.addData("Is running", limelight.isRunning());
            telemetry.addData("Status", status);

            if (result != null) {
                double[] pythonResult = result.getPythonOutput();

                if (pythonResult[4] == -1.0) {
                    tvec = Arrays.copyOfRange(pythonResult, 5, 8);

                    telemetry.addData("tvec", Arrays.toString(tvec));
                    telemetry.update();
                } else {
                    telemetry.addData("previous tvec", Arrays.toString(tvec));
                    telemetry.update();
                }
            } else {
                telemetry.addLine("No valid result D:");
                telemetry.addData("previous tvec", Arrays.toString(tvec));
                telemetry.update();
            }

            sleep(50);
        }
    }
}
