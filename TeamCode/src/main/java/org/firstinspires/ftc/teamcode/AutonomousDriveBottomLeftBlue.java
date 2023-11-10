
package org.firstinspires.ftc.teamcode;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
public abstract class AutonomousDriveBottomLeftBlue extends LinearOpMode {


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
        public double DISTANCE1 = 96;

        @Override
        public void runOpMode() throws InterruptedException {
            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            //drive.turn(MathtoRadians(-90))

            Trajectory autonomousTrajectory = drive.trajectoryBuilder(new Pose2d(-35, 61))
                    .splineTo(new Vector2d(-60,61), Math.toRadians(0))
            //.strafeLeft(25)
                    .splineTo(new Vector2d(-60,61), Math.toRadians(0))
            //.forward(25)
                    .splineTo(new Vector2d(-60,61), Math.toRadians(0))
            //.strafeLeft(25)
                    .splineTo(new Vector2d(-60,61), Math.toRadians(0))
            //.back(25)
                    .splineTo(new Vector2d(-60,61), Math.toRadians(0))
            //.strafeLeft(45)
            .build();

//            TrajectoryBuilder trajectory = drive.trajectoryBuilder(new Pose2d());
//            trajectory.strafeLeft(DISTANCE1);
//
//            trajectory.build();


            waitForStart();

            while (isStopRequested()) return;

            drive.followTrajectory(autonomousTrajectory);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;
        }
    }
}


