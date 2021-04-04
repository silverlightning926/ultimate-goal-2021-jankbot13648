package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.WobbleGoal;

@Autonomous(name = "THIS IS THE AUTONOMOUS!!")
public class BlueRight_Autonomous extends LinearOpMode {

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        driveBase = new DriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);

        timer = new ElapsedTime();

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        //driveBase.setPoseEstimate(Constants.STARTING_POSE);

        telemetry.addData("x", driveBase.getPoseEstimate().getX());
        telemetry.addData("y", driveBase.getPoseEstimate().getY());
        telemetry.addData("heading", driveBase.getPoseEstimate().getHeading());
        telemetry.update();

        Trajectory traj0 = driveBase.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, -18), 0)
                .build();

        Trajectory traj1 = driveBase.trajectoryBuilder(traj0.end())
                .splineTo(new Vector2d(57.5, -8), 0)
                .build();

        Trajectory traj2 = driveBase.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(113.32, 20.67, 1.6292))
                .build();

        Trajectory traj3 = driveBase.trajectoryBuilder(traj2.end())
                .back(6)
                .build();

        Trajectory traj4 = driveBase.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(35.785, 20.014, 4.6716))
                .build();

        Trajectory traj5 = driveBase.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(29.785, 18.014, 4.6716))
                .build();

        Trajectory traj6 = driveBase.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(20.059586793191045, 5.124632812288327, Math.toRadians(357.66438090824806)))
                .build();


        Trajectory traj7 = driveBase.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(38.028004165631287, 2.523665520064312, Math.toRadians(358.47773335618814)))
                .build();


        Trajectory traj8 = driveBase.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(41.108323, 21.665, 5.85648))
                .build();


        Trajectory traj9 = driveBase.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(56.321, 9.896, 6.05))
                .build();

        Trajectory traj10 = driveBase.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(101.42439, 20.53, 1.602))
                .build();


        Trajectory traj11 = driveBase.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(101.42439-5, 20.53, 1.602))
                .build();

        wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

        waitForStart();

        wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);

        shooter.SetShooter(Constants.SHOOTER_VELOCITY);
        intake.ReleaseWalls();
        intake.SetWallPosition(0.5, 0.3);

        while (!isStopRequested())
        {
            driveBase.followTrajectory(traj0);
            driveBase.followTrajectory(traj1);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            for(int i = 0; i < 3; i++)
            {
                shooter.Kick();

                sleep(300);

                shooter.Unkick();

                sleep(300);
            }

            driveBase.followTrajectory(traj2);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive())
            {}

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.75 && opModeIsActive())
            {}

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            timer.reset();

            while (timer.seconds() < 0.75 && opModeIsActive())
            {}

            driveBase.followTrajectory(traj3);

            driveBase.followTrajectory(traj4);

            driveBase.followTrajectory(traj5);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

            timer.reset();

            while (timer.seconds() < 1 && opModeIsActive())
            {}

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive())
            {}

            driveBase.followTrajectory(traj6);

            intake.SetIntake(1, 0);

            driveBase.followTrajectory(traj7);

            driveBase.followTrajectory(traj8);

            intake.SetIntake(0,0);
            shooter.SetShooter(Constants.SHOOTER_VELOCITY*1.05);

            driveBase.followTrajectory(traj9);

            for(int i = 0; i < 3; i++)
            {
                shooter.Kick();

                sleep(300);

                shooter.Unkick();

                sleep(300);
            }

            driveBase.followTrajectory(traj10);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive())
            {}

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive())
            {}

            intake.SetWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_OUT);

            driveBase.followTrajectory(traj11);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive())
            {}

            requestOpModeStop();
        }

        PoseStorage.currentPose = driveBase.getPoseEstimate();
    }
}
