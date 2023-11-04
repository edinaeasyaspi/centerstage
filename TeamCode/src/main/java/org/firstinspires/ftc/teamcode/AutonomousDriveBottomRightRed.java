
package org.firstinspires.ftc.teamcode;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public abstract class AutonomousDriveBottomRightRed extends LinearOpMode {


    abstract class sleep {
        // Declare an abstract sleep method
        public void sleep() {
            try {
                Thread.sleep(1000); // Sleep for 1 second (1000 milliseconds)
            } catch (InterruptedException e) {
                //handle the interrupted exception if necessary
            }
        }
    }

    public class TurnTest extends LinearOpMode {
        public double DISTANCE1 = 25;

        public double DISTANCE2 = 25;

        public double DISTANCE3 = 45;

        public double DISTANCE4 = 25;

        public double DISTANCE5 = 25;


        @Override
        public void runOpMode() throws InterruptedException {
            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            TrajectoryBuilder trajectory = drive.trajectoryBuilder(new Pose2d());
            trajectory.forward(DISTANCE1);
            trajectory.strafeLeft(DISTANCE2);
            trajectory.forward(DISTANCE3);
            trajectory.strafeRight(DISTANCE4);
            trajectory.forward(DISTANCE5);
            trajectory.build();


            waitForStart();

            if (isStopRequested()) return;

            drive.followTrajectory(trajectory.build());

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;
        }
    }
}


