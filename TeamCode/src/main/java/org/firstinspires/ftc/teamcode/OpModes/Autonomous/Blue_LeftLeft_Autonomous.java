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
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.Blue.RightLeft_Paths.Blue_RightLeft_4RingPath.*;

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

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        while (!isStarted()) {
            ringPosition = vision.pipeline.position;
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.addData("Analysis", vision.pipeline.getAnalysis());
            telemetry.update();
        }

        wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
        shooter.setShooter(Constants.SHOOTER_VELOCITY);
        intake.releaseFunnels();
        intake.setWallPosition(0.1, 0.3);

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.NONE)) {

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            sleep(250);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            driveBase.followTrajectory(traj0_0ring);

            driveBase.followTrajectory(traj1_0ring);

            driveBase.followTrajectory(traj2_0ring);

            ShootPowerShots();

            //drop the wobble goal

            driveBase.followTrajectory(traj3_0ring);


            wobbleGoal.setWobbleGoalManipulatorClose();

            sleep(750);

            wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            driveBase.followTrajectory(traj4_0ring);

            Shoot();


            driveBase.followTrajectory(traj5_0ring);

            //rotate the robot
            driveBase.turnTo(180);

            wobbleGoal.setWobbleGoalManipulatorOpen();

            sleep(500);



            //PoseStorage.currentPose = new Pose2d(driveBase.getPoseEstimate().getX(), driveBase.getPoseEstimate().getY(), Math.toRadians(90));
            requestOpModeStop();
        }

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.ONE)) {




            //PoseStorage.currentPose = new Pose2d(driveBase.getPoseEstimate().getX(), driveBase.getPoseEstimate().getY(), Math.toRadians(180));
            requestOpModeStop();

        }

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.FOUR)) {
        }
    }
     private void ShootPowerShots() {

        shooter.setShooter(Constants.POWER_SHOT_VELOCITY);
        for (int i = 0; i < 1; i++) {
            shooter.kick();

            sleep(250);

            shooter.unKick();

            sleep(250);
        }

        shooter.kick();

        sleep(250);

        shooter.unKick();

         shooter.setShooter(Constants.SHOOTER_VELOCITY);
    }

    private void Shoot() {
        for (int i = 0; i < 1; i++) {
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
