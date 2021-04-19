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
public class Blue_LeftLeft_4RingPath {

    public static Trajectory BLL4_traj1 = BuildTrajectory(new Pose2d())
            .lineToSplineHeading(new Pose2d(58, 0, Math.toRadians(342)))
            .build();

    public static Trajectory BLL4_traj2 = BuildTrajectory(BLL4_traj1.end())
            .lineToSplineHeading(new Pose2d(118.5, -8, 0))
            .build();

    public static Trajectory BLL4_traj3 = BuildTrajectory(BLL4_traj2.end())
            .lineToSplineHeading(new Pose2d(11.5,1, 0))
            .build();

    public static Trajectory BLL4_traj4 = BuildTrajectory(BLL4_traj3.end())
            .lineToConstantHeading(new Vector2d(11.5, -23.5))
            .build();

    public static Trajectory BLL4_traj5 = BuildTrajectory(BLL4_traj4.end())
            .lineToSplineHeading(new Pose2d(29, -22.5, Math.toRadians(353)),
                    DriveBase.getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH),
                    DriveBase.getAccelerationConstraint(MAX_ACCEL))
            .build();

    public static Trajectory BLL4_traj6 = BuildTrajectory(BLL4_traj5.end())
            .lineToConstantHeading(new Vector2d(40, -22.5),
                    DriveBase.getVelocityConstraint(5.5, MAX_ANG_VEL, TRACK_WIDTH),
                    DriveBase.getAccelerationConstraint(MAX_ACCEL))
            .build();

    public static Trajectory BLL4_traj7 = BuildTrajectory(BLL4_traj6.end())
            .lineToSplineHeading(new Pose2d(120.5, -10, Math.toRadians(90)))
            .build();

    public static Trajectory BLL4_traj8 = BuildTrajectory(BLL4_traj7.end())
            .lineToSplineHeading(new Pose2d(90, -10, Math.toRadians(90)))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
