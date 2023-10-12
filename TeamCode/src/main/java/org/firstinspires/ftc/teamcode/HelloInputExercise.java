package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class HelloInputExercise extends OpMode{
    float speed = -gamepad1.left_stick_y * 0.5F;
    public void init () {
        // Empty
    }
    public void loop () {
        while (gamepad1.a) {
            speed = -gamepad1.left_stick_y * 1F;
        }
    }
}
