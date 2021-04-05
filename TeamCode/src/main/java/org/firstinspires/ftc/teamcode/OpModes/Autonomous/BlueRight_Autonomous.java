package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Vision;
import org.firstinspires.ftc.teamcode.Systems.WobbleGoal;

@Autonomous(name = "BLUE - RIGHT")
public class BlueRight_Autonomous extends LinearOpMode {

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;
    Vision vision;

    ElapsedTime timer;

    Vision.SkystoneDeterminationPipeline.RingPosition ringPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        driveBase = new DriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);
        vision = new Vision(hardwareMap);

        timer = new ElapsedTime();

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        // 0 Ring Trajectories Start----------------------------------------------------------------

        Trajectory traj1_0ring = driveBase.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(10)))
                .build();

        // Shoot

        // Set Wobble Goal Position [1]

        Trajectory traj2_0ring = driveBase.trajectoryBuilder(traj1_0ring.end())
                .lineToSplineHeading(new Pose2d(61, 24, 1.6292))
                .build();

        // Open Wobble Goal Manipulator
        // Wait

        Trajectory traj3_0ring = driveBase.trajectoryBuilder(traj2_0ring.end())
                .back(6)
                .build();

        // Set Wobble Goal Position [2]

        Trajectory traj4_0ring = driveBase.trajectoryBuilder(traj3_0ring.end())
                .lineToSplineHeading(new Pose2d(31.785, 18.014, 4.6716))
                .build();

        // Close Wobble Goal Manipulator
        // Wait

        // Set Wobble Goal Position [1]

        Trajectory traj5_0ring = driveBase.trajectoryBuilder(traj4_0ring.end())
                .lineToSplineHeading(new Pose2d(55, 18.8, 1.6292))
                .build();

        // Open Wobble Goal Manipulator

        Trajectory traj6_0ring = driveBase.trajectoryBuilder(traj5_0ring.end())
                .back(10)
                .build();

        Trajectory traj7_0ring = driveBase.trajectoryBuilder(traj6_0ring.end())
                .lineToSplineHeading(new Pose2d(70, 0, 0))
                .build();

        // 0 Ring Trajectories End------------------------------------------------------------------

        // 4 Ring Trajectories Start----------------------------------------------------------------

        Trajectory traj1_4ring = driveBase.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(30, -18))
                .build();

        Trajectory traj2_4ring = driveBase.trajectoryBuilder(traj1_4ring.end())
                .lineToConstantHeading(new Vector2d(57.5, -8))
                .build();

        Trajectory traj3_4ring = driveBase.trajectoryBuilder(traj2_4ring.end())
                .lineToSplineHeading(new Pose2d(109.7, 18.8, 1.6292))
                .build();

        Trajectory traj4_4ring = driveBase.trajectoryBuilder(traj3_4ring.end())
                .back(6)
                .build();

        Trajectory traj5_4ring = driveBase.trajectoryBuilder(traj4_4ring.end())
                .lineToSplineHeading(new Pose2d(35.785, 22.014, 4.6716))
                .build();

        Trajectory traj6_4ring = driveBase.trajectoryBuilder(traj5_4ring.end())
                .lineToSplineHeading(new Pose2d(31.785, 18.014, 4.6716))
                .build();

        Trajectory traj7_4ring = driveBase.trajectoryBuilder(traj6_4ring.end())
                .lineToSplineHeading(new Pose2d(20.059586793191045, 5.124632812288327, Math.toRadians(357.66438090824806)))
                .build();

        Trajectory traj8_4ring = driveBase.trajectoryBuilder(traj7_4ring.end())
                .lineToSplineHeading(new Pose2d(41.108323, 2.523665520064312, 5.85648))
                .build();

        Trajectory traj09_4ring = driveBase.trajectoryBuilder(traj8_4ring.end())
                .lineToSplineHeading(new Pose2d(56.321, 9.896, 6.05))
                .build();

        Trajectory traj10_4ring = driveBase.trajectoryBuilder(traj09_4ring.end())
                .lineToSplineHeading(new Pose2d(98.42439, 17.0, Math.toRadians(90)))
                .build();

        Trajectory traj11_4ring = driveBase.trajectoryBuilder(traj10_4ring.end())
                .lineToSplineHeading(new Pose2d(94.42439, 17.0, Math.toRadians(90)))
                .build();

        // 4 Ring Trajectories End------------------------------------------------------------------

        wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

        while (!isStarted())
        {
            ringPosition = vision.pipeline.position;
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.update();

        }

        // For Testing Vision Rings
        /*while (!isStopRequested() && opModeIsActive())
        {
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.update();
        }*/

        wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
        shooter.SetShooter(Constants.SHOOTER_VELOCITY);
        intake.ReleaseWalls();
        intake.SetWallPosition(0.5, 0.3);

        while (!isStopRequested() && ringPosition.equals(Vision.SkystoneDeterminationPipeline.RingPosition.NONE))
        {
            telemetry.addData("Path", "1");
            telemetry.update();
            driveBase.followTrajectory(traj1_0ring);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(300);

                shooter.Unkick();

                sleep(300);
            }

            shooter.Kick();

            sleep(300);

            shooter.Unkick();

            telemetry.addData("Path", "2");
            telemetry.update();
            driveBase.followTrajectory(traj2_0ring);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            timer.reset();
            while (timer.seconds() < 0.5 && opModeIsActive());
            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);
            timer.reset();
            while (timer.seconds() < 0.50 && opModeIsActive());

            telemetry.addData("Path", "3");
            telemetry.update();
            driveBase.followTrajectory(traj3_0ring);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            telemetry.addData("Path", "4");
            telemetry.update();
            driveBase.followTrajectory(traj4_0ring);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

            timer.reset();
            while (timer.seconds() < 0.50 && opModeIsActive());

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            telemetry.addData("Path", "5");
            telemetry.update();
            driveBase.followTrajectory(traj5_0ring);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            timer.reset();
            while (timer.seconds() < 0.5 && opModeIsActive());
            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);
            timer.reset();
            while (timer.seconds() < 0.50 && opModeIsActive());

            telemetry.addData("Path", "6");
            telemetry.update();
            driveBase.followTrajectory(traj6_0ring);

            telemetry.addData("Path", "7");
            telemetry.update();
            driveBase.followTrajectory(traj7_0ring);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);
            timer.reset();
            while (timer.seconds() < 0.5 && opModeIsActive());

            PoseStorage.currentPose = new Pose2d(70, 0, Math.toRadians(90));
            requestOpModeStop();
        }

        while (!isStopRequested() && ringPosition.equals(Vision.SkystoneDeterminationPipeline.RingPosition.FOUR))
        {
            driveBase.followTrajectory(traj1_4ring);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj2_4ring);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(300);

                shooter.Unkick();

                sleep(300);
            }

            shooter.Kick();

            sleep(300);

            shooter.Unkick();

            driveBase.followTrajectory(traj3_4ring);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            timer.reset();

            while (timer.seconds() < 0.50 && opModeIsActive());

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.25 && opModeIsActive());

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            driveBase.followTrajectory(traj4_4ring);

            driveBase.followTrajectory(traj5_4ring);

            driveBase.followTrajectory(traj6_4ring);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

            timer.reset();

            while (timer.seconds() < 0.25 && opModeIsActive());

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj7_4ring);

            intake.SetIntake(1, 0);

            driveBase.followTrajectory(traj8_4ring);

            intake.SetIntake(0,0);
            shooter.SetShooter(Constants.SHOOTER_VELOCITY*1.05);

            driveBase.followTrajectory(traj09_4ring);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(300);

                shooter.Unkick();

                sleep(300);
            }

            shooter.Kick();

            sleep(300);

            shooter.Unkick();

            intake.SetWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj10_4ring);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive());

            driveBase.followTrajectory(traj11_4ring);

            PoseStorage.currentPose = new Pose2d(94.42439, 17.0, Math.toRadians(180));
            requestOpModeStop();
        }

        FtcDashboard.getInstance().stopCameraStream();
    }
}
