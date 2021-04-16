package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.Blue.RightLeft_Paths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.TRACK_WIDTH;

@Config
public class Blue_RightLeft_0RingPath {

    public static Trajectory BRL0_traj1 = BuildTrajectory(new Pose2d())
            .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(0)))
            .build();

    public static Trajectory BRL0_traj2 = BuildTrajectory(BRL0_traj1.end())
            .lineToSplineHeading(new Pose2d(61, 24, 1.6292))
            .build();


    public static Trajectory BRL0_traj3 = BuildTrajectory(BRL0_traj2.end())
            .lineToSplineHeading(new Pose2d(55, 18, 1.6292))
            .build();

    public static Trajectory BRL0_traj4 = BuildTrajectory(BRL0_traj3.end())
            .back(6)
            .build();

    public static Trajectory BRL0_traj5 = BuildTrajectory(BRL0_traj4.end())
            .lineToSplineHeading(new Pose2d(30, 19.75, 4.6716))
            .build();

    public static Trajectory BRL0_traj6 = BuildTrajectory(BRL0_traj5.end())
            .lineToSplineHeading(new Pose2d(26.585, 19.25, 4.6716))
            .build();

    public static Trajectory BRL0_traj7 = BuildTrajectory(BRL0_traj6.end())
            .lineToSplineHeading(new Pose2d(26.585, 12.25, 4.6716))
            .build();

    public static Trajectory BRL0_traj8 = BuildTrajectory(BRL0_traj7.end())
            .lineToSplineHeading(new Pose2d(55, 18.8, 1.6292))
            .build();

    public static Trajectory BRL0_traj9 = BuildTrajectory(BRL0_traj8.end())
            .lineToSplineHeading(new Pose2d(49, 18.8, 1.6292))
            .build();

    public static Trajectory BRL0_traj10 = BuildTrajectory(BRL0_traj9.end())
            .lineToSplineHeading(new Pose2d(40, 0, 0))
            .build();

    public static Trajectory BRL0_traj11 = BuildTrajectory(BRL0_traj10.end())
            .lineToSplineHeading(new Pose2d(70, 0, 0))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
