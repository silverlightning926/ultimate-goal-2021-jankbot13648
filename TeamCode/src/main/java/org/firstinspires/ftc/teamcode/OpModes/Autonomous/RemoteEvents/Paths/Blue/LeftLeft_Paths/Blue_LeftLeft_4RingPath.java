package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RemoteEvents.Paths.Blue.LeftLeft_Paths;

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

public class Blue_LeftLeft_4RingPath {

    public static Trajectory BLL4_traj1 = BuildTrajectory(new Pose2d())
            .lineToSplineHeading(new Pose2d(58, 0, Math.toRadians(339.75)))
            .build();

    public static Trajectory BLL4_traj2 = BuildTrajectory(BLL4_traj1.end())
            .lineToSplineHeading(new Pose2d(120.5, -5, 0))
            .build();

    public static Trajectory BLL4_traj3 = BuildTrajectory(BLL4_traj2.end())
            .lineToSplineHeading(new Pose2d(11.5,-1, 0))
            .build();

    public static Trajectory BLL4_traj4 = BuildTrajectory(BLL4_traj3.end())
            .lineToConstantHeading(new Vector2d(11.75, -24))
            .build();

    public static Trajectory BLL4_traj5 = BuildTrajectory(BLL4_traj4.end())
            .lineToSplineHeading(new Pose2d(28, -22.5, Math.toRadians(357.35)),
                    DriveBase.getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH),
                    DriveBase.getAccelerationConstraint(MAX_ACCEL))
            .build();

    public static Trajectory BLL4_traj6 = BuildTrajectory(BLL4_traj5.end())
            .back(2)
            .build();

    public static Trajectory BLL4_traj7 = BuildTrajectory(BLL4_traj6.end())
            .lineToConstantHeading(new Vector2d(42, -22.5),
                    DriveBase.getVelocityConstraint(3.5, MAX_ANG_VEL, TRACK_WIDTH),
                    DriveBase.getAccelerationConstraint(MAX_ACCEL))
            .build();

    public static Trajectory BLL4_traj8 = BuildTrajectory(BLL4_traj7.end())
            .lineToSplineHeading(new Pose2d(123.5, 3, Math.toRadians(90)))
            .build();

    public static Trajectory BLL4_traj9 = BuildTrajectory(BLL4_traj8.end())
            .lineToSplineHeading(new Pose2d(92, 1, Math.toRadians(90)))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
