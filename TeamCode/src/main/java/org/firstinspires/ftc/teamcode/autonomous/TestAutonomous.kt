package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive


class TestAutonomous : Methods() {
    override fun runOpMode() {
        val startPose = Pose2d(0.0, 0.0, 0.0)
        val drive = MecanumDrive(hardwareMap, startPose)

        val tab1 = drive.actionBuilder(startPose)
            .lineToX(10.0)
            .waitSeconds(5.0)
            .lineToX(0.0)

        val action1 = tab1.build()

        runBlocking(
            SequentialAction(
                action1
            )
        )
    }
}