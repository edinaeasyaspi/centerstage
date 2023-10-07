package org.firstinspires.ftc.teamcode;

// Imports a library or package full of classes.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// This is a comment.
/* This is another comment */
/*
* This
* is
* too */

/* This is a class.
* It has the same name as the file. */
/* @ Symbol indicates an annotation. These set mode of the game.
 * @TeleOp() To show up in the Driver Station as an OpMode.
 * @Autonomous() To show up im the Driver Station as a Self-Driving Mode.
 * @Override Serves to replace functionality in our parent class (to write our own).
 * @Disable
 */
// extends OpMode means that the class is a child of OpMode, thus getting its behavior.
@TeleOp()
public class HelloWorld extends OpMode {
    // The voids init() and loop() are required in an OpMode, and they have to be public.
    // We are using @Override to change the parent's behavior with our own.
    @Override
    // init() Method runs once when INIT is pressed.
    public void init(){
    // Telemetry - Sending information to the Driver Station. Caption is title and value is the text.
    telemetry.addData("Hello","World");
}
    @Override
    // loop() Runs run repeatedly after driver presses PLAY but before STOP.
    public void loop(){
    // Blank for now.
    }
}
