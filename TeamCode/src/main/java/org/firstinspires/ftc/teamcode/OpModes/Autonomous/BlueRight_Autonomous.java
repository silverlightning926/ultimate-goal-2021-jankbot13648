package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Vision;
import org.firstinspires.ftc.teamcode.Systems.WobbleGoal;

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

    Vision.RingDeterminationPipeline.RingPosition ringPosition;

    @Override
    public void runOpMode() {

        driveBase = new DriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);
        vision = new Vision(hardwareMap);

        wobbleGoal.setWobbleGoalManipulatorClose();

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        while (!isStarted())
        {
            ringPosition = vision.pipeline.position;
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.addData("Analysis", vision.pipeline.getAnalysis());
            telemetry.update();
        }

        wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
        shooter.setShooter(Constants.SHOOTER_VELOCITY);
        intake.releaseFunnels();
        intake.setWallPosition(0.1, 0.3);

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.NONE))
        {
            driveBase.followTrajectory(traj1_0ring);

            intake.setWallPosIn();

            sleep(1000);

            Shoot();

            driveBase.followTrajectory(traj2_0ring);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(500);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(500);

            driveBase.followTrajectory(traj2_1_0ring);

            driveBase.followTrajectory(traj3_0ring);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            driveBase.followTrajectory(traj4_0ring);

            driveBase.followTrajectory(traj4_1_0ring);

            wobbleGoal.setWobbleGoalManipulatorClose();

            sleep(750);

            driveBase.followTrajectory(traj4_2_0ring);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj5_0ring);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            sleep(500);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            sleep(500);

            driveBase.followTrajectory(traj5_1_0ring);

            driveBase.followTrajectory(traj6_0ring);

            driveBase.followTrajectory(traj7_0ring);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            sleep(500);

            PoseStorage.currentPose = new Pose2d(70, 0, Math.toRadians(90));
            requestOpModeStop();
        }

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.ONE))
        {
            // Follow Trajectories
            driveBase.followTrajectory(traj1_1ring);

            intake.setWallPosIn();

            driveBase.followTrajectory(traj2_1ring);

            Shoot();

            driveBase.followTrajectory(traj3_1ring);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(250);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(250);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            driveBase.followTrajectory(traj4_1ring);

            driveBase.followTrajectory(traj5_1ring);

            driveBase.followTrajectory(traj6_1ring);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(750);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj7_1ring);

            intake.setIntake(1, 0);

            intake.setWallPosIn();

            driveBase.followTrajectory(traj8_1ring);
            driveBase.followTrajectory(traj09_1ring);

            shooter.setShooter(Constants.SHOOTER_VELOCITY*1.01);

            driveBase.followTrajectory(traj10_1ring);
            intake.setIntake(0,0);

            Shoot();

            intake.setWallPosition(0.5, 0.3);

            driveBase.followTrajectory(traj11_1ring);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(500);

            intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj12_1ring);

            PoseStorage.currentPose = new Pose2d(94.42439, 17.0, Math.toRadians(180));
            requestOpModeStop();

        }

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.FOUR))
        {
            driveBase.followTrajectory(traj1_4ring);

            intake.setWallPosIn();

            driveBase.followTrajectory(traj2_4ring);

            Shoot();

            driveBase.followTrajectory(traj3_4ring);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(250);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(250);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(450);


            driveBase.followTrajectory(traj4_4ring);

            driveBase.followTrajectory(traj5_4ring);

            driveBase.followTrajectory(traj6_4ring);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(750);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj7_4ring);

            intake.setIntake(1, 0);

            intake.setWallPosIn();

            driveBase.followTrajectory(traj8_4ring);
            driveBase.followTrajectory(traj09_4ring);

            shooter.setShooter(Constants.SHOOTER_VELOCITY*1.01);

            driveBase.followTrajectory(traj10_4ring);
            intake.setIntake(0,0);

            Shoot();

            intake.setWallPosition(0.5, 0.3);

            driveBase.followTrajectory(traj11_4ring);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(500);

            intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(traj12_4ring);

            PoseStorage.currentPose = new Pose2d(94.42439, 17.0, Math.toRadians(180));
            requestOpModeStop();
        }

        FtcDashboard.getInstance().stopCameraStream();
    }

    private void Shoot() {
        for (int i = 0; i < 2; i++) {
            shooter.kick();

            sleep(250);

            shooter.unKick();

            sleep(250);
        }

        shooter.kick();

        sleep(250);

        shooter.unKick();
    }
}
