package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareTestBot
{
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HardwareTestBot() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        frontLeftDrive = hwMap.dcMotor.get ("frontLeft_drive");
        frontRightDrive = hwMap.dcMotor.get ("frontRight_drive");
        backLeftDrive = hwMap.dcMotor.get ("backLeft_drive");
        backRightDrive = hwMap.dcMotor.get ("backRight_drive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive
    }

}
