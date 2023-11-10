
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
public class AutonomousDriveBottomLeftBlue extends LinearOpMode {
        public double DISTANCE1 = 96;

        @Override
        public void runOpMode() throws InterruptedException {
            Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

           // drive.turn(Math.toRadians(-90));

            Trajectory autonomousTrajectory = drive.trajectoryBuilder(new Pose2d(-35,61))
                    .splineTo(new Vector2d(-60,61), Math.toRadians(0))
                    .splineTo(new Vector2d(-60,86), Math.toRadians(0))
                    .splineTo(new Vector2d(-85,86), Math.toRadians(0))
                    .splineTo(new Vector2d(-85,61), Math.toRadians(0))
                    .splineTo(new Vector2d(-110,61), Math.toRadians(0))

                    //.strafeLeft(25)
                    // .forward(25)
                    // .strafeLeft(25)
                    // .back(25)
                    //.strafeLeft(45)
                    .build();


//            TrajectoryBuilder trajectory = drive.trajectoryBuilder(new Pose2d());
//            trajectory.strafeLeft(DISTANCE1);
//            trajectory.build();


            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                drive.followTrajectory(autonomousTrajectory);
            }


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) ;
        }
    }
//}


