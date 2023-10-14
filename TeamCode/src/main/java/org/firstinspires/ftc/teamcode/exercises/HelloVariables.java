package org.firstinspires.ftc.teamcode.exercises;
// We import the packages.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class HelloVariables extends OpMode {
    @Override
    public void init() {
        /* Variables
        * int is for non decimal numbers 1 2 3 4...
        * double is for decimal numbers 1.2345...
        * boolean is for true or false.
        * String is for text.
         */
        int teamNumber = 8034;
        double motorSpeed = 0.5;
        boolean touchSensorPressed = true;
        String myName = "bimbimbambam";
        int grade = 10;

        // We output the variables.
        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor Pressed", touchSensorPressed);
        telemetry.addData("Hello", myName);
        telemetry.addData("Grade", grade);

        // A variable is only usable within its scope, inside of {}.
    }
    @Override
    public void loop() {
    // Oh, I won't let you go.
    }
}
