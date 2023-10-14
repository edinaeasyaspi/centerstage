package org.firstinspires.ftc.teamcode.exercises.tbd;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class HelloInputExercise extends OpMode{
    // Y stick is inverted
    float speedY = -gamepad1.left_stick_y * 0.5F;
    float speedX = gamepad1.left_stick_x * 0.5F;

    @Override
    public void init () {
        // Empty
    }
    @Override
    public void loop () {
        while (gamepad1.a) {
            speedY = -gamepad1.left_stick_y * 1F;
            speedX = -gamepad1.left_stick_y * 1F;
        }
        // EXERCISE 2 TBD (PAGE 27, exercise 4.6)
    }
}
