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

public class Blue_LeftLeft_1RingPath {

    public static Trajectory BLL1_traj1 = BuildTrajectory(new Pose2d())
            .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(323.98125)),
                DriveBase.getVelocityConstraint(60, Math.toRadians(60), TRACK_WIDTH),
                DriveBase.getAccelerationConstraint(50))
            .build();

    public static Trajectory BLL1_traj2 = BuildTrajectory(BLL1_traj1.end())
            .lineToSplineHeading(new Pose2d(94,-47,0))
            .build();

    public static Trajectory BLL1_traj3 = BuildTrajectory(BLL1_traj2.end())
            .lineToSplineHeading(new Pose2d(120,-35, Math.toRadians(0)))
            .build();

    public static Trajectory BLL1_traj4 = BuildTrajectory(new Pose2d(BLL1_traj2.end().getX(), BLL1_traj2.end().getY(), Math.toRadians(270)))
            .lineToConstantHeading(new Vector2d(122, -40))
            .build();

    public static Trajectory BLL1_traj5 = BuildTrajectory(BLL1_traj4.end())
            .lineToConstantHeading(new Vector2d(112.5,-67))
            .build();

    public static Trajectory BLL1_traj6 = BuildTrajectory(BLL1_traj5.end())
            .lineToSplineHeading(new Pose2d(18,-22, Math.toRadians(250)))
            .build();

    public static Trajectory BLL1_traj7 = BuildTrajectory(BLL1_traj6.end())
            .lineToSplineHeading(new Pose2d(13,-30, Math.toRadians(250)))
            .build();

    public static Trajectory BLL1_traj8 = BuildTrajectory(BLL1_traj7.end())
            .lineToSplineHeading(new Pose2d(3.5,-17.5, Math.toRadians(250)))
            .build();

    public static Trajectory BLL1_traj9 = BuildTrajectory(BLL1_traj8.end())
            .lineToSplineHeading(new Pose2d(15,-6, Math.toRadians(355)))
            .build();

    public static Trajectory BLL1_traj10 = BuildTrajectory(BLL1_traj9.end())
            .lineToSplineHeading(new Pose2d(60,-9, Math.toRadians(351)))
            .build();

    public static Trajectory BLL1_traj11 = BuildTrajectory(BLL1_traj10.end())
            .lineToSplineHeading(new Pose2d(60,-35, Math.toRadians(90)))
            .build();

    public static Trajectory BLL1_traj12 = BuildTrajectory(BLL1_traj11.end())
            .lineToSplineHeading(new Pose2d(90,-22, Math.toRadians(90)))
            .build();

    public static Trajectory BLL1_traj13 = BuildTrajectory(BLL1_traj12.end())
            .lineToSplineHeading(new Pose2d(80,-20, Math.toRadians(90)))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
