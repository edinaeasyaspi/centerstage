package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class BetterArcadeDrive extends OpMode {
    TwoMotorDrive drive = new TwoMotorDrive();

    @Override
    public void init()  {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x / 2;
        drive.setPowers(forward + right, forward - right);
    }
}
