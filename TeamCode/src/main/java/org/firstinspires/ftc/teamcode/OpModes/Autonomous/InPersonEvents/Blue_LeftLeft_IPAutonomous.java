package org.firstinspires.ftc.teamcode.OpModes.Autonomous.InPersonEvents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
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

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.TRACK_WIDTH;

@Autonomous(name = "Blue >> Left >> Left (In-Person)")
public class Blue_LeftLeft_IPAutonomous extends LinearOpMode {

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;
    RingDetectionCamera ringDetectionCamera;

    RingDeterminationPipeline.RingPosition ringPosition;

    @Override
    public void runOpMode() throws InterruptedException {

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
            ringPosition = ringDetectionCamera.getValue();
            telemetry.addData("Amount Of Rings", ringPosition);
            telemetry.addData("Analysis", ringDetectionCamera.getAnalysis());
            telemetry.update();
        }

        shooter.unKick();
        wobbleGoal.setWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

        intake.releaseFunnels();
        intake.setWallPosition(0.8, 0.3);

        Trajectory traj1 = BuildTrajectory(new Pose2d())
                .lineToSplineHeading(new Pose2d(58, 0, Math.toRadians(339.75)))
                .build();

        driveBase.followTrajectory(traj1);

        Shoot(3);

        if (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.NONE))
        {
            Trajectory IPBLL0_traj1 = BuildTrajectory(traj1.end())
                    .lineToSplineHeading(new Pose2d(64, 0, Math.toRadians(339.75)))
                    .build();

            driveBase.followTrajectory(IPBLL0_traj1);

            wobbleGoal.setWobbleGoalAutoClawOpen();
        }

        if (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.ONE))
        {
            Trajectory IPBLL1_traj1 = BuildTrajectory(traj1.end())
                    .lineToSplineHeading(new Pose2d(94,-47,0))
                    .build();

            driveBase.followTrajectory(IPBLL1_traj1);
            wobbleGoal.setWobbleGoalAutoClawOpen();

            Trajectory IPBLL1_traj2 = BuildTrajectory(IPBLL1_traj1.end())
                    .lineToSplineHeading(new Pose2d(64, 0, Math.toRadians(339.75)))
                    .build();

            driveBase.followTrajectory(IPBLL1_traj2);
        }

        if (!isStopRequested() && ringPosition.equals(RingDeterminationPipeline.RingPosition.FOUR))
        {
            Trajectory IPBLL4_traj1 = BuildTrajectory(traj1.end())
                    .lineToSplineHeading(new Pose2d(120.5, -5, 0))
                    .build();

            driveBase.followTrajectory(IPBLL4_traj1);
            wobbleGoal.setWobbleGoalAutoClawOpen();

            Trajectory IPBLL1_traj2 = BuildTrajectory(IPBLL4_traj1.end())
                    .lineToSplineHeading(new Pose2d(64, 0, Math.toRadians(339.75)))
                    .build();

            driveBase.followTrajectory(IPBLL1_traj2);
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

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
