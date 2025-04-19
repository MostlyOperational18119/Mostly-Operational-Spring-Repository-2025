package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Methods

@TeleOp(name = "Motor Tester")
class MotorTester : Methods() {
    override fun runOpMode() {

        initMotors()
        waitForStart()

        while (opModeIsActive()) {
            val driverCurrent = gamepad1
            val driverPrevious = gamepad1
            val gunnerCurrent = gamepad2
            val gunnerPrevious = gamepad2

            driverCurrent.copy(driverPrevious)
            gamepad1.copy(driverCurrent)

            gunnerCurrent.copy(gunnerPrevious)
            gamepad2.copy(gunnerCurrent)

            val up = gamepad1.y
            val down = gamepad1.a

            vertSlide!!.power = if (up) 0.1 else if (down) -0.1 else 0.0

            telemetry.addLine("Running")
            telemetry.addLine("Encoder Position: ${vertSlide!!.currentPosition}")
            telemetry.update()
        }
    }
}