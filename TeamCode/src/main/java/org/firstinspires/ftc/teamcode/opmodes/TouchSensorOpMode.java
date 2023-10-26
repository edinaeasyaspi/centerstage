package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;
@TeleOp()
public class TouchSensorOpMode extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    @Override
    public void init(){
    board.init(hardwareMap);
    }
    @Override
    public void loop() {
    telemetry.addData("Touch Sensor", board.getTouchSensorState());
    telemetry.addData("Touch Sensor Pressed", board.isTouchSensorPressed());
    telemetry.addData("Touch Sensor Released", board.isTouchSensorReleased());
    }
}
