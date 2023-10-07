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


    }


}
