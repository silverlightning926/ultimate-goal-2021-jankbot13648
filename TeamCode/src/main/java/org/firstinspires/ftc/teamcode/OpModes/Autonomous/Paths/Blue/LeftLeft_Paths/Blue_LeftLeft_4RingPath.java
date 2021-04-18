package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths.Blue.LeftLeft_Paths;

import android.os.Build;

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
public class Blue_LeftLeft_4RingPath {
    public static Trajectory BLL4_traj1 = BuildTrajectory(new Pose2d())
            .lineToSplineHeading(new Pose2d(58, 0, Math.toRadians(342)))
            .build();

    //change  to angle, to turn to the goal

    //kick the shooter

     public static Trajectory BLL4_traj2 = BuildTrajectory(BLL4_traj1.end())
             .lineToSplineHeading(new Pose2d(118.5, -8, 0))
             .build();

     //open the auto wobble claw


    //come back for the second wobble goal
    public static Trajectory BLL4_traj3 = BuildTrajectory(BLL4_traj2.end())
            .lineToSplineHeading(new Pose2d(11.5,1, 0))
            .build();


    public static Trajectory BLL4_traj3_1 = BuildTrajectory(BLL4_traj3.end())
            .lineToConstantHeading(new Vector2d(11.5, -23.5))
            .build();

    public static Trajectory BLL4_traj4 = BuildTrajectory(BLL4_traj3_1.end())
            .lineToSplineHeading(new Pose2d(29, -22.5, Math.toRadians(353)),new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                            )),
                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .build();

    //kick the shooter once
    //start the intake again
    public static Trajectory BLL4_traj5 = BuildTrajectory(BLL4_traj4.end())
            .lineToConstantHeading(new Vector2d(40, -22.5),new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(MAX_ANG_VEL),
                                    new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                            )),
                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .build();

    //kick the shooter 3 more times
    public static Trajectory BLL4_traj6 = BuildTrajectory(BLL4_traj5.end())
            .lineToSplineHeading(new Pose2d(120.5, -10, Math.toRadians(90)))
            .build();

    public static Trajectory BLL4_traj7 = BuildTrajectory(BLL4_traj6.end())
            .lineToSplineHeading(new Pose2d(90, -10, Math.toRadians(90)))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
