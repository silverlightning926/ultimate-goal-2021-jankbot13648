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
            .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(325.875)),
                    new MinVelocityConstraint(
                            Arrays.asList(
                                    new AngularVelocityConstraint(Math.toRadians(60)),
                                    new MecanumVelocityConstraint(MAX_VEL, DriveConstants.TRACK_WIDTH)
                            )),
                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
            .build();

    //change  to angle, to turn to the goal

    //kick the shooter

     public static Trajectory BLL4_traj2 = BuildTrajectory(BLL4_traj1.end())
             .splineToConstantHeading(new Vector2d(120, 24), Math.toRadians(0))
             .build();

     //open the auto wobble claw


    //come back for the second wobble goal
    public static Trajectory BLL4_traj3 = BuildTrajectory(BLL4_traj2.end())
            .splineToConstantHeading(new Vector2d(0,-24), Math.toRadians(0))
            .build();

    //pickup the second wobble goal

    //start to intake

    public static Trajectory BLL4_traj4 = BuildTrajectory(BLL4_traj3.end())
            .lineToConstantHeading(new Vector2d(24, 0))
            .build();

    //kick the shooter once
    //start the intake again
    public static Trajectory BLL4_traj5 = BuildTrajectory(BLL4_traj4.end())
            .lineToConstantHeading(new Vector2d(48, 0))
            .build();

    //kick the shooter 3 more times
    public static Trajectory BLL4_traj6 = BuildTrajectory(BLL4_traj5.end())
            .lineToConstantHeading(new Vector2d( 112, 0))
            .build();


    //rotate the robot 180
    //drop of the wobble goal

    //lastly park on the line
    public static Trajectory BLL4_traj7 = BuildTrajectory(BLL4_traj6.end())
            .lineToConstantHeading(new Vector2d( 72, 0))
            .build();

    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
