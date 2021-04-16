package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.Blue.LeftLeft_Paths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.TRACK_WIDTH;

@Config
public class Blue_LeftLeft_0RingPath
{
    public static Trajectory traj0_0ring = BuildTrajectory(new Pose2d())
            .strafeRight(5)
            .build();

    public static Trajectory traj1_0ring = BuildTrajectory(traj0_0ring.end())
            .forward(48)
            .build();

     public static Trajectory traj2_0ring = BuildTrajectory(traj1_0ring.end())
             .lineToSplineHeading(new Pose2d(48.001, 0, 45))
             .build();

     //kick the shooter
    //drop the wobble goal

    public static Trajectory traj3_0ring = BuildTrajectory(traj2_0ring.end())
            .splineToConstantHeading(new Vector2d(24, -23), Math.toRadians(0))
            .build();


    //pick up wobble goal #2

    public static Trajectory traj4_0ring = BuildTrajectory(traj3_0ring.end())
            .forward(12)
            .build();

    //shoot
    //pace the wobble goal

    public static Trajectory traj5_0ring = BuildTrajectory(traj4_0ring.end())
            .forward(2)
            .build();


    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
