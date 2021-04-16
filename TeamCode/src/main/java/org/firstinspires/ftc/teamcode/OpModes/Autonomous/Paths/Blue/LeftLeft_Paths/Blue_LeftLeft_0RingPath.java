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

import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.TRACK_WIDTH;

@Config
public class Blue_LeftLeft_0RingPath {
    public static Trajectory BLL0_traj1_0ring = BuildTrajectory(new Pose2d())
            .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(325.875)),
                    new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(Math.toRadians(60)),
                                    new MecanumVelocityConstraint(MAX_VEL, DriveConstants.TRACK_WIDTH)
                            )),
                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .build();

    public static Trajectory BLL0_traj2_0ring = BuildTrajectory(BLL0_traj1_0ring.end())
            .lineToSplineHeading(new Pose2d(114,6, Math.toRadians(315)))
            .build();

    public static Trajectory BLL0_traj3_0ring = BuildTrajectory(BLL0_traj2_0ring.end())
            .lineToSplineHeading(new Pose2d(114,-48, Math.toRadians(315)))
            .build();


    public static Trajectory BLL0_traj4_0ring = BuildTrajectory(BLL0_traj3_0ring.end())
            .lineToSplineHeading(new Pose2d(24,-18, Math.toRadians(0)))
            .build();

    public static Trajectory BLL0_traj5_0ring = BuildTrajectory(BLL0_traj4_0ring.end())
            .lineToConstantHeading(new Vector2d(48,-18))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
