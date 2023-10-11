package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="TestBot: Simple Autonomous", group="TestBot")
public abstract class TestBotSimpleAuto extends LinearOpMode {
    PushbotAutoDriveByEncoder_Linear robot = new PushbotAutoDriveByEncoder_Linear();
    private ElapsedTime runtime = new ElapsedTime();

    static final double  FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    @Override
    public void  runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status","Ready to run");
        telemetry.update();

        waitForStart();
        
        robot.leftDrive.setPower(FORWARD_SPEED);
        robot.rightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path","Leg1: %2.5f S Elapsed" , runtime.seconds.());
            telemetry.update();
        }

        robot.leftDrive.setPower(TURN_SPEED);
        robot.rightDrive.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.leftDrive.setPower(-FORWARD_SPEED);
        robot.rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed")
            telemetry.update();
        }
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");

        waitForStart();

        robot.leftFrontMotor.setPower(1);
        robot.leftBackMotor.setPower(1);
        robot.rightFrontMotor.setPower(1);
        robot.rightBackMotor.setPower(1);
        sleep(1000);

        robot.leftFrontMotor.setPower(-0.5);
        robot.leftBackMotor.setPower(-0.5);
        robot.rightFrontMotor.setPower(-0.5);
        robot.rightBackMotor.setPower(-0.5);

        telemetry.addData("Path", "Complete")
                telemetry.update();
                sleep(1000);
    }


}
