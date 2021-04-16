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

        shooter.unKick();
        wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);
        shooter.setShooter(180);
        intake.releaseFunnels();
        intake.setWallPosition(0.8, 0.3);

        while (!isStopRequested() && ringPosition.equals(Vision.RingDeterminationPipeline.RingPosition.NONE)) {

            driveBase.followTrajectory(BLL0_traj1_0ring);
            intake.setWallPosIn();

            shooter.kick();
            sleep(250);
            shooter.unKick();
            sleep(250);

            shooter.setShooter(Constants.SHOOTER_VELOCITY);

            wobbleGoal.setWobbleGoalAutoClawOpen();

            driveBase.turnTo(Math.toRadians(345));

            shooter.kick();
            sleep(200);
            shooter.unKick();
            sleep(200);
            shooter.kick();
            sleep(200);
            shooter.unKick();

            driveBase.followTrajectory(BLL0_traj2_0ring);

            intake.setIntakeWithoutWalls(1);

            driveBase.followTrajectory(BLL0_traj3_0ring);

            intake.setIntakeWithoutWalls(0);

            driveBase.followTrajectory(BLL0_traj4_0ring);

            break;
        }

        requestOpModeStop();

        //PoseStorage.currentPose = driveBase.getPoseEstimate();
        //FtcDashboard.getInstance().stopCameraStream();
    }


    private void Shoot() {

        for (int i = 0; i < 2; i++) {
            shooter.kick();

            sleep(Constants.shooterDelay);

            shooter.unKick();

            sleep(Constants.dropDelay);
        }

        intake.setWallPosDown();

        shooter.kick();

        sleep(Constants.shooterDelay);

        shooter.unKick();
    }

}
