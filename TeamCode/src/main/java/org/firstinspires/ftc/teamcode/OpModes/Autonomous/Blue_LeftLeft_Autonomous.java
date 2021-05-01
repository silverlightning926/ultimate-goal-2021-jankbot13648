package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Vision.Pipelines.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.Systems.Vision.RingDetectionCamera;
import org.firstinspires.ftc.teamcode.Systems.WobbleGoal;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.Blue.LeftLeft_Paths.Blue_LeftLeft_0RingPath.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.Blue.LeftLeft_Paths.Blue_LeftLeft_1RingPath.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.Blue.LeftLeft_Paths.Blue_LeftLeft_4RingPath.*;

@Autonomous(name = "BLUE (LEFT >> LEFT)")
public class Blue_LeftLeft_Autonomous extends LinearOpMode {

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
        wobbleGoal.setWobbleGoalAutoClawClose();
        intake.closeFunnels();

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        while (!isStarted()) {
            ringPosition = ringDetectionCamera.pipeline.position;
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.addData("Analysis", ringDetectionCamera.pipeline.getAnalysis());
            telemetry.update();
        }

        shooter.unKick();
        wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

        if (ringPosition.equals(RingDeterminationPipeline.RingPosition.FOUR)) {
            shooter.setShooter(Constants.AUTONOMOUS_SHOOTER_SPEED);
        } else {
            shooter.setShooter(178);
        }

        intake.releaseFunnels();
        intake.setWallPosition(0.8, 0.3);

        if (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.NONE)) {

            driveBase.followTrajectory(BLL0_traj1);
            intake.setWallPosIn();
            sleep(500);

            Shoot(1);

            shooter.setShooter(176.5);

            driveBase.turnTo(Math.toRadians(332.5));

            Shoot(1);

            shooter.setShooter(Constants.AUTONOMOUS_SHOOTER_SPEED);
            sleep(250);

            driveBase.turnTo(Math.toRadians(343));

            Shoot(1);

            intake.setIntakeWithoutWalls(1);

            driveBase.followTrajectory(BLL0_traj2);

            wobbleGoal.setWobbleGoalAutoClawOpen();
            sleep(1000);

            driveBase.followTrajectory(BLL0_traj3);
            driveBase.followTrajectory(BLL0_traj4);

            driveBase.turnTo(Math.toRadians(300));
            driveBase.followTrajectory(BLL0_traj5);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            wobbleGoal.setWobbleGoalManipulatorOpen();

            driveBase.followTrajectory(BLL0_traj6);

            driveBase.followTrajectory(BLL0_traj7);

            intake.setIntakeWithoutWalls(0);

            wobbleGoal.setWobbleGoalManipulatorClose();
            sleep(350);
            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(BLL0_traj8);

            Shoot(3);

            driveBase.followTrajectory(BLL0_traj9);
            driveBase.followTrajectory(BLL0_traj10);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            sleep(500);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            sleep(250);
            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
            sleep(500);

            PoseStorage.currentPose = driveBase.getPoseEstimate();
            driveBase.followTrajectory(BLL0_traj11);
        }

        if (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.ONE)) {

            driveBase.followTrajectory(BLL1_traj1);
            intake.setWallPosIn();
            sleep(500);

            Shoot(1);

            shooter.setShooter(176.5);

            driveBase.turnTo(Math.toRadians(332.5));

            Shoot(1);

            sleep(250);

            shooter.setShooter(Constants.AUTONOMOUS_SHOOTER_SPEED);

            driveBase.turnTo(Math.toRadians(343));

            Shoot(1);

            driveBase.followTrajectory(BLL1_traj2);

            intake.setIntakeWithoutWalls(1);

            wobbleGoal.setWobbleGoalAutoClawOpen();
            sleep(250);

            driveBase.followTrajectory(BLL1_traj3);
            driveBase.followTrajectory(BLL1_traj4);

            wobbleGoal.setWobbleGoalManipulatorOpen();
            driveBase.followTrajectory(BLL1_traj5);
            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            driveBase.followTrajectory(BLL1_traj6);

            wobbleGoal.setWobbleGoalManipulatorClose();
            sleep(350);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(BLL1_traj7);
            driveBase.followTrajectory(BLL1_traj8);

            driveBase.followTrajectory(BLL1_traj9);

            sleep(500);

            Shoot(3);

            intake.setIntakeWithoutWalls(0);

            driveBase.followTrajectory(BLL1_traj10);
            driveBase.followTrajectory(BLL1_traj11);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
            sleep(375);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            driveBase.followTrajectory(BLL1_traj12);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);
            sleep(250);

            PoseStorage.currentPose = driveBase.getPoseEstimate();
        }

        if (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.FOUR))
        {
            driveBase.followTrajectory(BLL4_traj1);
            intake.setWallPosIn();
            sleep(500);

            Shoot(3);

            driveBase.followTrajectory(BLL4_traj2);
            wobbleGoal.setWobbleGoalAutoClawOpen();
            sleep(1000);

            shooter.setShooter(199);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            driveBase.followTrajectory(BLL4_traj3);
            driveBase.followTrajectory(BLL4_traj4);
            wobbleGoal.setWobbleGoalManipulatorClose();
            sleep(500);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
            intake.setIntakeWithoutWalls(1);
            driveBase.followTrajectory(BLL4_traj5);
            driveBase.followTrajectory(BLL4_traj6);

            sleep(1500);

            Shoot(2);

            shooter.setShooter(190.5);

            driveBase.followTrajectory(BLL4_traj7);

            sleep(1000);

            intake.setIntakeWithoutWalls(0);

            Shoot(3);

            driveBase.followTrajectory(BLL4_traj8);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            sleep(250);
            wobbleGoal.setWobbleGoalManipulatorOpen();

            intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);
            PoseStorage.currentPose = driveBase.getPoseEstimate();

            driveBase.followTrajectory(BLL4_traj9);
        }

        PoseStorage.currentPose = driveBase.getPoseEstimate();
        FtcDashboard.getInstance().stopCameraStream();

        requestOpModeStop();
    }

    public void Shoot(int numOfTimes)
    {
        for(int i = 0; i < numOfTimes - 1; i++)
        {
            shooter.kick();
            sleep(200);
            shooter.unKick();
            sleep(200);
        }

        shooter.kick();
        sleep(200);
        shooter.unKick();
    }
}
