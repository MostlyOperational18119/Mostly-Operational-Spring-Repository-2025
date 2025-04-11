package org.firstinspires.ftc.teamcode

class VeryGoodTeleOp : Methods() {
    override fun runOpMode() {
        val gamepad1Current = gamepad1
        val gamepad1Previous = gamepad1
        val gamepad2Current = gamepad2
        val gamepad2Previous = gamepad2

        val gamepad1LeftX = gamepad1.left_stick_x
        val gamepad1LeftY = -gamepad1.left_stick_y
        val gamepad1RightX = gamepad1.right_stick_x

        val speedDiv = 1.0

        initMotors()
        waitForStart()

        while (opModeIsActive()) {
            gamepad1Current.copy(gamepad1Previous)
            gamepad1.copy(gamepad1Current)

            gamepad2Current.copy(gamepad2Previous)
            gamepad2.copy(gamepad2Current)

            motorFL!!.power = (gamepad1LeftY + gamepad1LeftX + gamepad1RightX) / 1.0
            motorFR!!.power = (gamepad1LeftY - gamepad1LeftX + gamepad1RightX) / 1.0
            motorBL!!.power = (gamepad1LeftY - gamepad1LeftX - gamepad1RightX) / 1.0
            motorBR!!.power = (gamepad1LeftY + gamepad1LeftX - gamepad1RightX) / 1.0


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