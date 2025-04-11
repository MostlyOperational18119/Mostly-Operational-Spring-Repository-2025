package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

abstract class Methods : LinearOpMode() {
    var motorFL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorBR: DcMotor? = null

    fun initMotors() {
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]
    }
}