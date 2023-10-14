package org.firstinspires.ftc.teamcode.exercises;
// Packages yet again.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class HelloDecisions extends OpMode{
    @Override
    public void init(){
        // Once upon a time there was an init() method that wasn't used.
    }
    @Override
    public void loop() {
        // Decision making stuff. If something happens then the code runs.
        if (gamepad1.left_stick_x < 0) {
            telemetry.addData("Left stick", "is negative.");
        }
        else if (gamepad1.left_stick_x > 0) {
            telemetry.addData("Left stick", "is positive.");
        }
    }
}
