package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RemoteEvents.Paths.Blue.RightLeft_Paths;

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

public class Blue_RightLeft_1RingPath {

    public static Trajectory BRL1_traj1 = BuildTrajectory(new Pose2d())
            .lineToConstantHeading(new Vector2d(30, -18))
            .build();

    public static Trajectory BRL1_traj2 = BuildTrajectory(BRL1_traj1.end())
            .lineToConstantHeading(new Vector2d(55.5, -8))
            .build();

    public static Trajectory BRL1_traj3 = BuildTrajectory(BRL1_traj2.end())
            .lineToSplineHeading(new Pose2d(87.7,-5.2 , 1.6292))
            .build();

    public static Trajectory BRL1_traj4 = BuildTrajectory(BRL1_traj3.end())
            .back(6)
            .build();

    public static Trajectory BRL1_traj5 = BuildTrajectory(BRL1_traj4.end())
            .lineToSplineHeading(new Pose2d(35.785, 20, 4.6716))
            .build();

    public static Trajectory BRL1_traj6 = BuildTrajectory(BRL1_traj5.end())
            .lineToSplineHeading(new Pose2d(26.585, 19.25, 4.6716))
            .build();

    public static Trajectory BRL1_traj7 = BuildTrajectory(BRL1_traj6.end())
            .lineToSplineHeading(new Pose2d(18.059586793191045, 5.124632812288327, Math.toRadians(357.66438090824806)))
            .build();

    public static Trajectory BRL1_traj8 = BuildTrajectory(BRL1_traj7.end())
            .lineToSplineHeading(new Pose2d(35.108323, 2.523665520064312, 5.85648), new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(Math.toRadians(25)),
                                    new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                            )),
                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();

    public static Trajectory BRL1_traj9 = BuildTrajectory(BRL1_traj8.end())
            .lineToSplineHeading(new Pose2d(41.321, 20.0, 0.0))
            .build();

    public static Trajectory BRL1_traj10 = BuildTrajectory(BRL1_traj9.end())
            .lineToSplineHeading(new Pose2d(56.321, 20, Math.toRadians(350)))
            .build();

    public static Trajectory BRL1_traj11 = BuildTrajectory(BRL1_traj10.end())
            .lineToSplineHeading(new Pose2d(81.42439, -9.0, Math.toRadians(90)))
            .build();

    public static Trajectory BRL1_traj12 = BuildTrajectory(BRL1_traj11.end())
            .lineToSplineHeading(new Pose2d(74.42439, 17.0, Math.toRadians(90)))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
