package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.BlueRight_0RingPath.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.BlueRight_1RingPath.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.BlueRight_4RingPath.*;

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

        wobbleGoal.SetWobbleGoalManipulatorClose();

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

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

        wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
        shooter.SetShooter(Constants.SHOOTER_VELOCITY);
        intake.ReleaseFunnels();
        intake.SetWallPosition(0.1, 0.3);

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.NONE))
        {
            telemetry.addData("Path", "1");
            telemetry.update();
            driveBase.followTrajectory(traj1_0ring);

            intake.SetWallPosIn();

            WaitForSeconds(1.0);

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

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            WaitForSeconds(0.5);

            wobbleGoal.SetWobbleGoalManipulatorOpen();

            WaitForSeconds(0.50);

            driveBase.followTrajectory(traj2_1_0ring);

            telemetry.addData("Path", "3");
            telemetry.update();
            driveBase.followTrajectory(traj3_0ring);

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            telemetry.addData("Path", "4");
            telemetry.update();
            driveBase.followTrajectory(traj4_0ring);

            driveBase.followTrajectory(traj4_1_0ring);

            wobbleGoal.SetWobbleGoalManipulatorClose();

            WaitForSeconds(0.75);

            driveBase.followTrajectory(traj4_2_0ring);

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            telemetry.addData("Path", "5");
            telemetry.update();
            driveBase.followTrajectory(traj5_0ring);

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            WaitForSeconds(0.5);
            wobbleGoal.SetWobbleGoalManipulatorOpen();
            WaitForSeconds(0.50);

            driveBase.followTrajectory(traj5_1_0ring);

            telemetry.addData("Path", "6");
            telemetry.update();
            driveBase.followTrajectory(traj6_0ring);

            telemetry.addData("Path", "7");
            telemetry.update();
            driveBase.followTrajectory(traj7_0ring);

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
            wobbleGoal.SetWobbleGoalManipulatorOpen();
            WaitForSeconds(0.5);

            PoseStorage.currentPose = new Pose2d(70, 0, Math.toRadians(90));
            requestOpModeStop();
        }

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.ONE))
        {
            // Follow Trajectories
            driveBase.followTrajectory(traj1_1ring);

            intake.SetWallPosIn();

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

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            WaitForSeconds(0.25);

            wobbleGoal.SetWobbleGoalManipulatorOpen();

            WaitForSeconds(0.25);

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            driveBase.followTrajectory(traj4_1ring);

            driveBase.followTrajectory(traj5_1ring);

            driveBase.followTrajectory(traj6_1ring);

            wobbleGoal.SetWobbleGoalManipulatorOpen();

            WaitForSeconds(0.75);

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj7_1ring);

            intake.SetIntake(1, 0);

            intake.SetWallPosIn();

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

            wobbleGoal.SetWobbleGoalManipulatorOpen();

            WaitForSeconds(0.5);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj12_1ring);

            PoseStorage.currentPose = new Pose2d(94.42439, 17.0, Math.toRadians(180));
            requestOpModeStop();

        }

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.FOUR))
        {
            driveBase.followTrajectory(traj1_4ring);

            intake.SetWallPosIn();

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

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            WaitForSeconds(0.25);

            wobbleGoal.SetWobbleGoalManipulatorOpen();

            WaitForSeconds(0.25);

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            WaitForSeconds(0.45);


            driveBase.followTrajectory(traj4_4ring);

            driveBase.followTrajectory(traj5_4ring);

            driveBase.followTrajectory(traj6_4ring);

            wobbleGoal.SetWobbleGoalManipulatorOpen();

            WaitForSeconds(0.75);

            wobbleGoal.SetWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj7_4ring);

            intake.SetIntake(1, 0);

            intake.SetWallPosIn();

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

            wobbleGoal.SetWobbleGoalManipulatorOpen();

            WaitForSeconds(0.5);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj12_4ring);

            PoseStorage.currentPose = new Pose2d(94.42439, 17.0, Math.toRadians(180));
            requestOpModeStop();
        }

        FtcDashboard.getInstance().stopCameraStream();
    }

    private void WaitForSeconds(double v) {
        timer.reset();
        while (timer.seconds() < v && opModeIsActive()) ;
    }
}
