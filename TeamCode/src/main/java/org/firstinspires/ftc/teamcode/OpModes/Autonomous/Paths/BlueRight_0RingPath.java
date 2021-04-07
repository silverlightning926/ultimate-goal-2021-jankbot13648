package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths;

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
public class BlueRight_0RingPath {

    public static Trajectory traj1_0ring = BuildTrajectory(new Pose2d())
            .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(0)))
            .build();

    public static Trajectory traj2_0ring = BuildTrajectory(traj1_0ring.end())
            .lineToSplineHeading(new Pose2d(61, 24, 1.6292))
            .build();


    public static Trajectory traj2_1_0ring = BuildTrajectory(traj2_0ring.end())
            .lineToSplineHeading(new Pose2d(55, 18, 1.6292))
            .build();

    public static Trajectory traj3_0ring = BuildTrajectory(traj2_1_0ring.end())
            .back(6)
            .build();

    public static Trajectory traj4_0ring = BuildTrajectory(traj3_0ring.end())
            .lineToSplineHeading(new Pose2d(30, 19.75, 4.6716))
            .build();

    public static Trajectory traj4_1_0ring = BuildTrajectory(traj4_0ring.end())
            .lineToSplineHeading(new Pose2d(26.585, 19.25, 4.6716))
            .build();

    public static Trajectory traj4_2_0ring = BuildTrajectory(traj4_1_0ring.end())
            .lineToSplineHeading(new Pose2d(26.585, 12.25, 4.6716))
            .build();

    public static Trajectory traj5_0ring = BuildTrajectory(traj4_2_0ring.end())
            .lineToSplineHeading(new Pose2d(55, 18.8, 1.6292))
            .build();

    public static Trajectory traj5_1_0ring = BuildTrajectory(traj5_0ring.end())
            .lineToSplineHeading(new Pose2d(49, 18.8, 1.6292))
            .build();

    public static Trajectory traj6_0ring = BuildTrajectory(traj5_1_0ring.end())
            .lineToSplineHeading(new Pose2d(40, 0, 0))
            .build();

    public static Trajectory traj7_0ring = BuildTrajectory(traj6_0ring.end())
            .lineToSplineHeading(new Pose2d(70, 0, 0))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
