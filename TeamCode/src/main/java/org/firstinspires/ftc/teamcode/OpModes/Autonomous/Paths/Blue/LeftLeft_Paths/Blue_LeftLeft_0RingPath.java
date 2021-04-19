package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.Blue.LeftLeft_Paths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
public class Blue_LeftLeft_0RingPath {

    public static Trajectory BLL0_traj1 = BuildTrajectory(new Pose2d())
            .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(325.875)),
                DriveBase.getVelocityConstraint(MAX_VEL, Math.toRadians(60), TRACK_WIDTH),
                DriveBase.getAccelerationConstraint(MAX_ACCEL))
            .build();

    public static Trajectory BLL0_traj2 = BuildTrajectory(BLL0_traj1.end())
            .forward(5)
            .build();

    public static Trajectory BLL0_traj3 = BuildTrajectory(BLL0_traj2.end())
            .strafeRight(12)
            .build();

    public static Trajectory BLL0_traj4 = BuildTrajectory(BLL0_traj3.end())
            .lineToSplineHeading(new Pose2d(116,0, Math.toRadians(315)))
            .build();

    public static Trajectory BLL0_traj5 = BuildTrajectory(BLL0_traj4.end())
            .lineToConstantHeading(new Vector2d(112.5,-63))
            .build();

    public static Trajectory BLL0_traj6 = BuildTrajectory(BLL0_traj5.end())
            .lineToSplineHeading(new Pose2d(3.5,-16, Math.toRadians(0)))
            .build();

    public static Trajectory BLL0_traj7 = BuildTrajectory(BLL0_traj6.end())
            .lineToSplineHeading(new Pose2d(3.5,-22.5, Math.toRadians(0)))
            .build();

    public static Trajectory BLL0_traj8 = BuildTrajectory(BLL0_traj7.end())
            .lineToSplineHeading(new Pose2d(55, -19.5, Math.toRadians(353)))
            .build();

    public static Trajectory BLL0_traj9 = BuildTrajectory(BLL0_traj8.end())
            .lineToSplineHeading(new Pose2d(60, -19.5, Math.toRadians(180)))
            .build();

    public static Trajectory BLL0_traj10 = BuildTrajectory(BLL0_traj9.end())
            .lineToSplineHeading(new Pose2d(87, -7, Math.toRadians(180)))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
