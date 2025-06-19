package org.firstinspires.ftc.teamcode.services.Communication;

import com.qualcomm.robotcore.hardware.Gamepad;

public class TeleOpServiceInput {
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public TeleOpServiceInput(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}
