package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import org.firstinspires.ftc.teamcode.services.Communication.DriveServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.TeleOpServiceInput;
import org.firstinspires.ftc.teamcode.services.Communication.VisionServiceOutput;
import org.firstinspires.ftc.teamcode.services.Service.TeleOpService;

import java.util.concurrent.LinkedBlockingQueue;

public class FirstServiceTeleOpService extends TeleOpService {
    public FirstServiceTeleOpService(LinkedBlockingQueue<DriveServiceInput> driveServiceInputQueue, LinkedBlockingQueue<VisionServiceOutput> visionServiceOutputQueue, LinkedBlockingQueue<TeleOpServiceInput> teleOpServiceInputQueue) {
        super(driveServiceInputQueue, visionServiceOutputQueue, teleOpServiceInputQueue);
    }

    @Override
    public void run() {
        Double speedDivider = 1.0;

        while (true) {
            TeleOpServiceInput input = teleOpServiceInputQueue.poll();

            if (input != null) {
                if (input.gamepad1.a) {
                    Log.d("FirstServiceTeleOpService", "A button pressed");
                }

                DriveServiceInput output;

                if (input.gamepad1.left_bumper || input.gamepad1.right_bumper) {
                    output = new DriveServiceInput(true);
                } else {
                    output = new DriveServiceInput(
                            ((-input.gamepad1.left_stick_y + input.gamepad1.left_stick_x + input.gamepad1.right_stick_x) / speedDivider),
                            ((-input.gamepad1.left_stick_y - input.gamepad1.left_stick_x - input.gamepad1.right_stick_x) / speedDivider),
                            ((-input.gamepad1.left_stick_y - input.gamepad1.left_stick_x + input.gamepad1.right_stick_x) / speedDivider),
                            ((-input.gamepad1.left_stick_y + input.gamepad1.left_stick_x - input.gamepad1.right_stick_x) / speedDivider),
                            0.0,
                            0.0
                    );
                }

                driveServiceInputQueue.add(output);
            } else {
                Log.e("FirstServiceTeleOpService", "input is null D:");
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // Womp womp
            }
        }
    }
}
