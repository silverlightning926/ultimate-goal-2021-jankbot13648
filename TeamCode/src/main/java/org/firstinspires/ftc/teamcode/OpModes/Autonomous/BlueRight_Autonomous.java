package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Vision;
import org.firstinspires.ftc.teamcode.Systems.WobbleGoal;

import java.util.Arrays;

@Autonomous(name = "BLUE - RIGHT")
public class BlueRight_Autonomous extends LinearOpMode {

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;
    Vision vision;

    ElapsedTime timer;

    Vision.RingDeterminationPipeline.RingPosition ringPosition;

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
                .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(0)))
                .build();

        Trajectory traj2_0ring = driveBase.trajectoryBuilder(traj1_0ring.end())
                .lineToSplineHeading(new Pose2d(61, 24, 1.6292))
                .build();


        Trajectory traj2_1_0ring = driveBase.trajectoryBuilder(traj2_0ring.end())
                .lineToSplineHeading(new Pose2d(55, 18, 1.6292))
                .build();

        Trajectory traj3_0ring = driveBase.trajectoryBuilder(traj2_1_0ring.end())
                .back(6)
                .build();

        Trajectory traj4_0ring = driveBase.trajectoryBuilder(traj3_0ring.end())
                .lineToSplineHeading(new Pose2d(30, 19.75, 4.6716))
                .build();

        Trajectory traj4_1_0ring = driveBase.trajectoryBuilder(traj4_0ring.end())
                .lineToSplineHeading(new Pose2d(26.585, 19.25, 4.6716))
                .build();

        Trajectory traj4_2_0ring = driveBase.trajectoryBuilder(traj4_1_0ring.end())
                .lineToSplineHeading(new Pose2d(26.585, 12.25, 4.6716))
                .build();


        Trajectory traj5_0ring = driveBase.trajectoryBuilder(traj4_2_0ring.end())
                .lineToSplineHeading(new Pose2d(55, 18.8, 1.6292))
                .build();


        Trajectory traj5_1_0ring = driveBase.trajectoryBuilder(traj5_0ring.end())
                .lineToSplineHeading(new Pose2d(49, 18.8, 1.6292))
                .build();

        Trajectory traj6_0ring = driveBase.trajectoryBuilder(traj5_1_0ring.end())
                .lineToSplineHeading(new Pose2d(40, 0, 0))
                .build();

        Trajectory traj7_0ring = driveBase.trajectoryBuilder(traj6_0ring.end())
                .lineToSplineHeading(new Pose2d(70, 0, 0))
                .build();

        // 0 Ring Trajectories End------------------------------------------------------------------

        // 1 Ring Trajectories Start----------------------------------------------------------------

        Trajectory traj1_1ring = driveBase.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(30, -18))
                .build();

        Trajectory traj2_1ring = driveBase.trajectoryBuilder(traj1_1ring.end())
                .lineToConstantHeading(new Vector2d(55.5, -8))
                .build();

        Trajectory traj3_1ring = driveBase.trajectoryBuilder(traj2_1ring.end())
                .lineToSplineHeading(new Pose2d(87.7,-5.2 , 1.6292))
                .build();

        Trajectory traj4_1ring = driveBase.trajectoryBuilder(traj3_1ring.end())
                .back(6)
                .build();

        Trajectory traj5_1ring = driveBase.trajectoryBuilder(traj4_1ring.end())
                .lineToSplineHeading(new Pose2d(35.785, 20, 4.6716))
                .build();

        Trajectory traj6_1ring = driveBase.trajectoryBuilder(traj5_1ring.end())
                .lineToSplineHeading(new Pose2d(26.585, 21.25, 4.6716))
                .build();

        Trajectory traj7_1ring = driveBase.trajectoryBuilder(traj6_1ring.end())
                .lineToSplineHeading(new Pose2d(18.059586793191045, 5.124632812288327, Math.toRadians(357.66438090824806)))
                .build();

        Trajectory traj8_1ring = driveBase.trajectoryBuilder(traj7_1ring.end())
                .lineToSplineHeading(new Pose2d(35.108323, 2.523665520064312, 5.85648), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(25),
                                        new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                )),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj09_1ring = driveBase.trajectoryBuilder(traj8_1ring.end())
                .lineToSplineHeading(new Pose2d(41.321, 20.0, 0.0))
                .build();

        Trajectory traj10_1ring = driveBase.trajectoryBuilder(traj09_1ring.end())
                .lineToSplineHeading(new Pose2d(56.321, 20, 5.85))
                .build();

        Trajectory traj11_1ring = driveBase.trajectoryBuilder(traj10_1ring.end())
                .lineToSplineHeading(new Pose2d(81.42439, -9.0, Math.toRadians(90)))
                .build();

        Trajectory traj12_1ring = driveBase.trajectoryBuilder(traj11_1ring.end())
                .lineToSplineHeading(new Pose2d(74.42439, 17.0, Math.toRadians(90)))
                .build();

        // 1 Ring Trajectories End------------------------------------------------------------------

        // 4 Ring Trajectories Start----------------------------------------------------------------

        Trajectory traj1_4ring = driveBase.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(30, -18))
                .build();

        Trajectory traj2_4ring = driveBase.trajectoryBuilder(traj1_4ring.end())
                .lineToConstantHeading(new Vector2d(55.5, -8))
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
                .lineToSplineHeading(new Pose2d(26.585, 21.25, 4.6716))
                .build();

        Trajectory traj7_4ring = driveBase.trajectoryBuilder(traj6_4ring.end())
                .lineToSplineHeading(new Pose2d(18.059586793191045, 5.124632812288327, Math.toRadians(357.66438090824806)))
                .build();

        Trajectory traj8_4ring = driveBase.trajectoryBuilder(traj7_4ring.end())
                .lineToSplineHeading(new Pose2d(35.108323, 2.523665520064312, 5.85648), new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(25),
                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory traj09_4ring = driveBase.trajectoryBuilder(traj8_4ring.end())
                .lineToSplineHeading(new Pose2d(41.321, 20.0, 0.0))
                .build();

        Trajectory traj10_4ring = driveBase.trajectoryBuilder(traj09_4ring.end())
                .lineToSplineHeading(new Pose2d(56.321, 20, 5.85))
                .build();

        Trajectory traj11_4ring = driveBase.trajectoryBuilder(traj10_4ring.end())
                .lineToSplineHeading(new Pose2d(98.42439, 15.0, Math.toRadians(90)))
                .build();

        Trajectory traj12_4ring = driveBase.trajectoryBuilder(traj11_4ring.end())
                .lineToSplineHeading(new Pose2d(92.42439, 17.0, Math.toRadians(90)))
                .build();

        // 4 Ring Trajectories End------------------------------------------------------------------

        wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

        while (!isStarted())
        {
            ringPosition = vision.pipeline.position;
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.addData("Analysis", vision.pipeline.getAnalysis());
            telemetry.update();

        }

        // For Testing Vision Rings
        /*while (!isStopRequested() && opModeIsActive())
        {
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.update();
        }*/

        wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
        shooter.SetShooter(Constants.SHOOTER_VELOCITY);
        intake.ReleaseWalls();
        intake.SetWallPosition(0.1, 0.3);

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.NONE))
        {
            telemetry.addData("Path", "1");
            telemetry.update();
            driveBase.followTrajectory(traj1_0ring);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            timer.reset();
            while (timer.seconds() < 1.0 && opModeIsActive());

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

            driveBase.followTrajectory(traj2_1_0ring);

            telemetry.addData("Path", "3");
            telemetry.update();
            driveBase.followTrajectory(traj3_0ring);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            telemetry.addData("Path", "4");
            telemetry.update();
            driveBase.followTrajectory(traj4_0ring);

            driveBase.followTrajectory(traj4_1_0ring);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

            timer.reset();
            while (timer.seconds() < 0.75 && opModeIsActive());

            driveBase.followTrajectory(traj4_2_0ring);

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

            driveBase.followTrajectory(traj5_1_0ring);

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

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.ONE))
        {
            // Follow Trajectories
            driveBase.followTrajectory(traj1_1ring);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj2_1ring);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(275);

                shooter.Unkick();

                sleep(275);
            }

            shooter.Kick();

            sleep(275);

            shooter.Unkick();

            driveBase.followTrajectory(traj3_1ring);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            timer.reset();

            while (timer.seconds() < 0.25 && opModeIsActive());

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.25 && opModeIsActive());

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            driveBase.followTrajectory(traj4_1ring);

            driveBase.followTrajectory(traj5_1ring);

            driveBase.followTrajectory(traj6_1ring);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

            timer.reset();

            while (timer.seconds() < 0.75 && opModeIsActive());

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj7_1ring);

            intake.SetIntake(1, 0);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj8_1ring);
            driveBase.followTrajectory(traj09_1ring);

            shooter.SetShooter(Constants.SHOOTER_VELOCITY*1.01);

            driveBase.followTrajectory(traj10_1ring);
            intake.SetIntake(0,0);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(250);

                shooter.Unkick();

                sleep(250);
            }

            shooter.Kick();

            sleep(250);

            shooter.Unkick();

            intake.SetWallPosition(0.5, 0.3);

            driveBase.followTrajectory(traj11_1ring);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive());

            intake.SetWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj12_1ring);

            PoseStorage.currentPose = new Pose2d(94.42439, 17.0, Math.toRadians(180));
            requestOpModeStop();

        }

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.FOUR))
        {
            driveBase.followTrajectory(traj1_4ring);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj2_4ring);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(275);

                shooter.Unkick();

                sleep(275);
            }

            shooter.Kick();

            sleep(275);

            shooter.Unkick();

            driveBase.followTrajectory(traj3_4ring);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            timer.reset();

            while (timer.seconds() < 0.25 && opModeIsActive());

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.25 && opModeIsActive());

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            timer.reset();
            while (timer.seconds() < 0.45 && opModeIsActive());


            driveBase.followTrajectory(traj4_4ring);

            driveBase.followTrajectory(traj5_4ring);

            driveBase.followTrajectory(traj6_4ring);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

            timer.reset();

            while (timer.seconds() < 0.75 && opModeIsActive());

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj7_4ring);

            intake.SetIntake(1, 0);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj8_4ring);
            driveBase.followTrajectory(traj09_4ring);

            shooter.SetShooter(Constants.SHOOTER_VELOCITY*1.01);

            driveBase.followTrajectory(traj10_4ring);
            intake.SetIntake(0,0);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(250);

                shooter.Unkick();

                sleep(250);
            }

            shooter.Kick();

            sleep(250);

            shooter.Unkick();

            intake.SetWallPosition(0.5, 0.3);

            driveBase.followTrajectory(traj11_4ring);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive());

            intake.SetWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj12_4ring);

            PoseStorage.currentPose = new Pose2d(94.42439, 17.0, Math.toRadians(180));
            requestOpModeStop();
        }

        FtcDashboard.getInstance().stopCameraStream();
    }
}
