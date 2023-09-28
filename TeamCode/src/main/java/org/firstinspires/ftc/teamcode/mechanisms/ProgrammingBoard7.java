package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProgrammingBoard7 {
    private DigitalChannel touchSensor;
    private DcMotor motor;
    private double ticksPerRotation;
private Servo servo;
private AnalogInput pot;
private ColorSensor colorSensor;
private DistanceSensor distanceSensor;

public void init (HardwareMap hwMap) {
    distanceSensor = hwMap.get(DistanceSensor.class, "distance_sensor");

}

    public double getDistance(DistanceUnit du){
         return distanceSensor.getDistance(du);
    }

}
