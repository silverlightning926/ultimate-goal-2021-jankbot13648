package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Vision;
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
        wobbleGoal.setWobbleGoalAutoClawClose();
        intake.closeFunnels();

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        while (!isStarted()) {
            ringPosition = vision.pipeline.position;
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.addData("Analysis", vision.pipeline.getAnalysis());
            telemetry.update();
        }

        shooter.unKick();
        wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

        if (ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.FOUR)) {
            shooter.setShooter(Constants.SHOOTER_VELOCITY);
        } else {
            shooter.setShooter(180.5);
        }

        intake.releaseFunnels();
        intake.setWallPosition(0.8, 0.3);

        if (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.NONE)) {

            driveBase.followTrajectory(BLL0_traj1);
            intake.setWallPosIn();

            Shoot(1);

            shooter.setShooter(179.5);

            driveBase.turnTo(Math.toRadians(333));

            Shoot(1);

            shooter.setShooter(Constants.SHOOTER_VELOCITY);

            driveBase.turnTo(Math.toRadians(345));

            Shoot(1);

            intake.setIntakeWithoutWalls(1);

            driveBase.followTrajectory(BLL0_traj2);

            wobbleGoal.setWobbleGoalAutoClawOpen();
            sleep(1000);

            driveBase.followTrajectory(BLL0_traj3);
            driveBase.followTrajectory(BLL0_traj4);
            driveBase.followTrajectory(BLL0_traj5);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            wobbleGoal.setWobbleGoalManipulatorOpen();

            driveBase.followTrajectory(BLL0_traj6);

            driveBase.followTrajectory(BLL0_traj7);

            intake.setIntakeWithoutWalls(0);

            wobbleGoal.setWobbleGoalManipulatorClose();
            sleep(1000);
            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(BLL0_traj8);

            Shoot(3);

            driveBase.followTrajectory(BLL0_traj9);
            driveBase.followTrajectory(BLL0_traj10);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            sleep(500);

            driveBase.followTrajectory(BLL0_traj3);
        }

        if (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.ONE)) {

            driveBase.followTrajectory(BLL1_traj1);
            intake.setWallPosIn();

            Shoot(1);

            shooter.setShooter(179.5);

            driveBase.turnTo(Math.toRadians(333));

            Shoot(1);

            shooter.setShooter(Constants.SHOOTER_VELOCITY);

            driveBase.turnTo(Math.toRadians(345));

            Shoot(1);

            driveBase.followTrajectory(BLL1_traj2);

            wobbleGoal.setWobbleGoalAutoClawOpen();
            sleep(500);

            intake.setIntakeWithoutWalls(1);
            driveBase.followTrajectory(BLL1_traj3);
            driveBase.followTrajectory(BLL1_traj4);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            driveBase.followTrajectory(BLL1_traj5);

            intake.setIntakeWithoutWalls(0);

            driveBase.followTrajectory(BLL1_traj6);

            wobbleGoal.setWobbleGoalManipulatorClose();
            sleep(500);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(BLL1_traj7);
            driveBase.followTrajectory(BLL1_traj8);

            intake.setIntakeWithoutWalls(1);
            driveBase.followTrajectory(BLL1_traj9);

            sleep(1000);

            Shoot(3);

            intake.setIntakeWithoutWalls(0);

            driveBase.followTrajectory(BLL1_traj10);
            driveBase.followTrajectory(BLL1_traj11);

            wobbleGoal.setWobbleGoalManipulatorOpen();
            intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);
            sleep(500);

            driveBase.followTrajectory(BLL1_traj12);
        }

        if (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.FOUR))
        {
            driveBase.followTrajectory(BLL4_traj1);
            intake.setWallPosIn();

            Shoot(3);

            driveBase.followTrajectory(BLL4_traj2);
            wobbleGoal.setWobbleGoalAutoClawOpen();
            sleep(1000);

            shooter.setShooter(202);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);
            wobbleGoal.setWobbleGoalManipulatorOpen();
            driveBase.followTrajectory(BLL4_traj3);
            driveBase.followTrajectory(BLL4_traj4);
            wobbleGoal.setWobbleGoalManipulatorClose();
            sleep(500);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
            intake.setIntakeWithoutWalls(1);
            driveBase.followTrajectory(BLL4_traj5);

            sleep(1500);

            Shoot(2);

            shooter.setShooter(193);

            driveBase.followTrajectory(BLL4_traj6);

            sleep(1000);

            intake.setIntakeWithoutWalls(0);

            Shoot(3);

            driveBase.followTrajectory(BLL4_traj7);
            wobbleGoal.setWobbleGoalManipulatorOpen();

            intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);
            driveBase.followTrajectory(BLL4_traj8);
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
            sleep(180);
            shooter.unKick();
            sleep(180);
        }

        shooter.kick();
        sleep(180);
        shooter.unKick();
    }
}
