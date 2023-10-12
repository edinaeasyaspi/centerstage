package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Deprecated
@TeleOp()
public class HelloWorldLinear extends LinearOpMode {

    @Override
    public void runOpMode () {
        telemetry.addData("Hello","World");
        telemetry.update();
        waitForStart();
    }
}