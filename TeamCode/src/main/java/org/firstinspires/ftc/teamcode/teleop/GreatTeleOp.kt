package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Methods

@TeleOp(name = "Great TeleOp")
class GreatTeleOp : Methods() {
    override fun runOpMode() {
        val driverCurrent = gamepad1
        val driverPrevious = gamepad1
        val gunnerCurrent = gamepad2
        val gunnerPrevious = gamepad2

        val driverLeftX = gamepad1.left_stick_x
        val driverLeftY = -gamepad1.left_stick_y
        val driverRightX = gamepad1.right_stick_x

        val gunnerRightY = gamepad2.right_stick_y
        val gunnerLeftX = gamepad2.left_stick_x

        val speedDiv = 1.0

        initMotors()
        waitForStart()

        while (opModeIsActive()) {
            driverCurrent.copy(driverPrevious)
            gamepad1.copy(driverCurrent)

            gunnerCurrent.copy(gunnerPrevious)
            gamepad2.copy(gunnerCurrent)

            motorFL!!.power = (driverLeftY + driverLeftX + driverRightX) / speedDiv
            motorFR!!.power = (driverLeftY - driverLeftX + driverRightX) / speedDiv
            motorBL!!.power = (driverLeftY - driverLeftX - driverRightX) / speedDiv
            motorBR!!.power = (driverLeftY + driverLeftX - driverRightX) / speedDiv


            telemetry.addLine("Running")
            telemetry.addLine("Speed div: $speedDiv")
            telemetry.addLine("Motor FL: ${motorFL!!.power}")
            telemetry.addLine("Motor FR: ${motorFR!!.power}")
            telemetry.addLine("Motor BL: ${motorBL!!.power}")
            telemetry.addLine("Motor BR: ${motorBR!!.power}")
            telemetry.update()
        }
    }
}