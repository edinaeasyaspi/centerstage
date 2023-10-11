package org.firstinspires.ftc.teamcode;
// We import the packages.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class HelloGamepads extends OpMode {
    @Override
    public void init() {
        // It is spooky month.
    }
    @Override
    public void loop() {
        // We make a variable and add a basic math operator.
        // The output we want is the opposite as the one we code, thus adding the "-" sign.
        double speedForwardLeftStick = -gamepad1.left_stick_y / 2.0;
        double speedForwardRightStick = -gamepad1.right_stick_y / 2.0;
        // Sends the variable stored in the physical components.
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Left Stick x", gamepad1.left_stick_x);
        telemetry.addData("Speed Forward Left Stick", speedForwardLeftStick);
        telemetry.addData("Speed Forward Right Stick", speedForwardRightStick);
        telemetry.addData("B Button", gamepad1.b);
        telemetry.addData("Difference between left joystick y and right joystick y", gamepad1.left_trigger - gamepad1.right_trigger);
    }
}
