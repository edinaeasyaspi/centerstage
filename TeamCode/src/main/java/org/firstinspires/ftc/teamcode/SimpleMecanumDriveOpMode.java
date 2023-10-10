package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp()
public abstract class SimpleMecanumDriveOpMode extends OpMode {
    TwoMotorDrive drive = new TwoMotorDrive();

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1. left_stick_x;

        drive.setPowers(forward + right, forward - right);
    }
}

