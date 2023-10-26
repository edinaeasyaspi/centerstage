package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class ProgrammingBoard {
    private DigitalChannel touchSensor;

    public void init (HardwareMap hwMap) {
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean getTouchSensorState() {
        return touchSensor.getState();
    }
    public boolean isTouchSensorPressed() {
        if(!touchSensor.getState()) {
            return true; // If state is false, Touch Sensor is pressed.
        }
        return false;
    }
    public boolean isTouchSensorReleased() {
        if(touchSensor.getState()) {
            return false;
        }
        return true;
    }
}
