package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RemoteEvents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Vision.Pipelines.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.Systems.Vision.RingDetectionCamera;
import org.firstinspires.ftc.teamcode.Systems.WobbleGoal;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RemoteEvents.Paths.Blue.RightLeft_Paths.Blue_RightLeft_0RingPath.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RemoteEvents.Paths.Blue.RightLeft_Paths.Blue_RightLeft_1RingPath.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.RemoteEvents.Paths.Blue.RightLeft_Paths.Blue_RightLeft_4RingPath.*;

@Disabled
@Autonomous(name = "BLUE (RIGHT >> LEFT)")
public class Blue_RightLeft_Autonomous extends LinearOpMode {

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;
    RingDetectionCamera ringDetectionCamera;

    RingDeterminationPipeline.RingPosition ringPosition;

    @Override
    public void runOpMode() {

        driveBase = new DriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);
        ringDetectionCamera = new RingDetectionCamera(hardwareMap);

        wobbleGoal.setWobbleGoalManipulatorClose();

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        while (!isStarted())
        {
            ringPosition = ringDetectionCamera.getValue();
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.addData("Analysis", ringDetectionCamera.getAnalysis());
            telemetry.update();
        }

        wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
        shooter.setShooter(Constants.AUTONOMOUS_SHOOTER_SPEED);
        intake.releaseFunnels();
        intake.setWallPosition(0.1, 0.3);

        while (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.NONE))
        {
            driveBase.followTrajectory(BRL0_traj1);

            intake.setWallPosIn();

            sleep(1000);

            Shoot();

            driveBase.followTrajectory(BRL0_traj2);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(500);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(500);

            driveBase.followTrajectory(BRL0_traj3);

            driveBase.followTrajectory(BRL0_traj4);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            driveBase.followTrajectory(BRL0_traj5);

            driveBase.followTrajectory(BRL0_traj6);

            wobbleGoal.setWobbleGoalManipulatorClose();

            sleep(750);

            driveBase.followTrajectory(BRL0_traj7);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(BRL0_traj8);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            sleep(500);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            sleep(500);

            driveBase.followTrajectory(BRL0_traj9);

            driveBase.followTrajectory(BRL0_traj10);

            driveBase.followTrajectory(BRL0_traj11);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            sleep(500);

            //PoseStorage.currentPose = new Pose2d(driveBase.getPoseEstimate().getX(), driveBase.getPoseEstimate().getY(), Math.toRadians(90));
            requestOpModeStop();
        }

        while (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.ONE))
        {
            // Follow Trajectories
            driveBase.followTrajectory(BRL1_traj1);

            intake.setWallPosIn();

            driveBase.followTrajectory(BRL1_traj2);

            Shoot();

            driveBase.followTrajectory(BRL1_traj3);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(250);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(250);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            driveBase.followTrajectory(BRL1_traj4);

            driveBase.followTrajectory(BRL1_traj5);

            driveBase.followTrajectory(BRL1_traj6);

            wobbleGoal.setWobbleGoalManipulatorClose();

            sleep(750);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(BRL1_traj7);

            intake.setIntake(1, 0);

            intake.setWallPosIn();

            driveBase.followTrajectory(BRL1_traj8);
            driveBase.followTrajectory(BRL1_traj9);

            shooter.setShooter(Constants.AUTONOMOUS_SHOOTER_SPEED);

            driveBase.followTrajectory(BRL1_traj10);
            intake.setIntake(0,0);

            Shoot();

            intake.setWallPosition(0.5, 0.3);

            driveBase.followTrajectory(BRL1_traj11);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(500);

            intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(BRL1_traj12);

            //PoseStorage.currentPose = new Pose2d(driveBase.getPoseEstimate().getX(), driveBase.getPoseEstimate().getY(), Math.toRadians(180));
            requestOpModeStop();

        }

        while (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.FOUR))
        {
            driveBase.followTrajectory(BRL4_traj1);

            intake.setWallPosIn();

            driveBase.followTrajectory(BRL4_traj2);

            Shoot();

            driveBase.followTrajectory(BRL4_traj3);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(250);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(250);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(450);


            driveBase.followTrajectory(BRL4_traj4);

            driveBase.followTrajectory(BRL4_traj5);

            driveBase.followTrajectory(BRL4_traj6);

            wobbleGoal.setWobbleGoalManipulatorClose();

            sleep(750);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(BRL4_traj7);

            intake.setIntake(1, 0);

            intake.setWallPosIn();

            driveBase.followTrajectory(BRL4_traj8);
            driveBase.followTrajectory(BRL4_traj9);

            shooter.setShooter(Constants.AUTONOMOUS_SHOOTER_SPEED);

            driveBase.followTrajectory(BRL4_traj10);
            intake.setIntake(0,0);

            Shoot();

            intake.setWallPosition(0.5, 0.3);

            driveBase.followTrajectory(BRL4_traj11);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(500);

            intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(BRL4_traj12);

            requestOpModeStop();
        }

        PoseStorage.currentPose = driveBase.getPoseEstimate();
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
