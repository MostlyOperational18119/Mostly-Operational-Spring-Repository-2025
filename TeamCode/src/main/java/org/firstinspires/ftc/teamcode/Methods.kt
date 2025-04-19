package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

abstract class Methods : LinearOpMode() {
    var motorFL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorBR: DcMotor? = null
    var horSlide: DcMotor? = null
    var vertSlide: DcMotor? = null


    fun initMotors() {
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]
        horSlide = hardwareMap.dcMotor["horSlide"]
        horSlide!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        horSlide!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        vertSlide = hardwareMap.dcMotor["vertSlide"]
        vertSlide!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        vertSlide!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}