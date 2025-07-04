package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name="GoBilda PinPoint computer tester")
public class PinPointTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-170, 0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Init done :D");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            telemetry.addData(
                    "Pinpoint info",
                    String.format(
                            Locale.ENGLISH,
                            "(X %.5f) (Y %.5f) (Yaw %.1f, unnorm is %.1f) (X accel %.5f) (Y accel %.5f) (Yaw accel %.1f)",
                            pinpoint.getPosX(DistanceUnit.METER),
                            pinpoint.getPosY(DistanceUnit.METER),
                            pinpoint.getHeading(AngleUnit.DEGREES),
                            pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES),
                            pinpoint.getVelX(DistanceUnit.METER),
                            pinpoint.getVelY(DistanceUnit.METER),
                            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)
                    )
            );

            telemetry.update();

            sleep(30);
        }
    }
}
