package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths;

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
public class BlueRight_1RingPath {

    public static Trajectory traj1_1ring = BuildTrajectory(new Pose2d())
            .lineToConstantHeading(new Vector2d(30, -18))
            .build();

    public static Trajectory traj2_1ring = BuildTrajectory(traj1_1ring.end())
            .lineToConstantHeading(new Vector2d(55.5, -8))
            .build();

    public static Trajectory traj3_1ring = BuildTrajectory(traj2_1ring.end())
            .lineToSplineHeading(new Pose2d(87.7,-5.2 , 1.6292))
            .build();

    public static Trajectory traj4_1ring = BuildTrajectory(traj3_1ring.end())
            .back(6)
            .build();

    public static Trajectory traj5_1ring = BuildTrajectory(traj4_1ring.end())
            .lineToSplineHeading(new Pose2d(35.785, 20, 4.6716))
            .build();

    public static Trajectory traj6_1ring = BuildTrajectory(traj5_1ring.end())
            .lineToSplineHeading(new Pose2d(26.585, 21.25, 4.6716))
            .build();

    public static Trajectory traj7_1ring = BuildTrajectory(traj6_1ring.end())
            .lineToSplineHeading(new Pose2d(18.059586793191045, 5.124632812288327, Math.toRadians(357.66438090824806)))
            .build();

    public static Trajectory traj8_1ring = BuildTrajectory(traj7_1ring.end())
            .lineToSplineHeading(new Pose2d(35.108323, 2.523665520064312, 5.85648), new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(Math.toRadians(25)),
                                    new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                            )),
                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();

    public static Trajectory traj09_1ring = BuildTrajectory(traj8_1ring.end())
            .lineToSplineHeading(new Pose2d(41.321, 20.0, 0.0))
            .build();

    public static Trajectory traj10_1ring = BuildTrajectory(traj09_1ring.end())
            .lineToSplineHeading(new Pose2d(56.321, 20, 5.85))
            .build();

    public static Trajectory traj11_1ring = BuildTrajectory(traj10_1ring.end())
            .lineToSplineHeading(new Pose2d(81.42439, -9.0, Math.toRadians(90)))
            .build();

    public static Trajectory traj12_1ring = BuildTrajectory(traj11_1ring.end())
            .lineToSplineHeading(new Pose2d(74.42439, 17.0, Math.toRadians(90)))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
