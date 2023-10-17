package org.firstinspires.ftc.teamcode;     // password for the control hub is edina8034

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")

public class RobotAutoDriveByEncoder_Linear extends LinearOpMode {

    private DcMotor         frontLeftMotor   = null;
    private DcMotor         frontRightMotor = null;
    private DcMotor         backLeftMotor    = null;
    private DcMotor         backRightMotor  = null;

    private final ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 2880 ;    // eg: TETRIX Motor Encoder

    //Motor numbers per rotation: 1440;1, 2880;2, 4320;3, 5760;4, 7200;5, etc.
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.145);
    static final double     DRIVE_SPEED             = 1.0;

    //How fast the robot drives (max speed = 1)
    static final double     TURN_SPEED              = 0.5;

    //How fast the robot turns (max speed = 1)

    @Override
    public void runOpMode() {

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition());

        telemetry.update();

        telemetry.addData("Starting at",  "%7d :%7d",
                backLeftMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition());

        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED,  22,  22, 0.2);  // S1: Forward 22 Inches with 0.2 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100);  // pause to display final telemetry message.
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;


        if (opModeIsActive()) {
            
            newBackLeftTarget = backLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = backLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = backRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            frontLeftMotor.setTargetPosition(newBackLeftTarget);
            frontRightMotor.setTargetPosition(newBackRightTarget);
            backLeftMotor.setTargetPosition(newFrontLeftTarget);
            backRightMotor.setTargetPosition(newFrontRightTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            frontLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            backLeftMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    frontLeftMotor.isBusy() &&
                     frontRightMotor.isBusy() &&
                backLeftMotor.isBusy() &&
                    backRightMotor.isBusy())
            {

                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
                telemetry.update();
            }

            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(1);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
}
