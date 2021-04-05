package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.WobbleGoal;

import static org.firstinspires.ftc.teamcode.Constants.VUFORIA_LICENSE_KEY;

@Autonomous(name = "THIS IS THE AUTONOMOUS!!")
public class BlueRight_Autonomous extends LinearOpMode {

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;

    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        driveBase = new DriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);

        timer = new ElapsedTime();

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        Trajectory traj1 = driveBase.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(30, -18))
                .build();

        Trajectory traj2 = driveBase.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(57.5, -8))
                .build();

        Trajectory traj3 = driveBase.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(109.7, 18.8, 1.6292))
                .build();

        Trajectory traj4 = driveBase.trajectoryBuilder(traj3.end())
                .back(6)
                .build();

        Trajectory traj5 = driveBase.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(35.785, 22.014, 4.6716))
                .build();

        Trajectory traj6 = driveBase.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(31.785, 18.014, 4.6716))
                .build();

        Trajectory traj7 = driveBase.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(20.059586793191045, 5.124632812288327, Math.toRadians(357.66438090824806)))
                .build();

        Trajectory traj8 = driveBase.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(38.028004165631287, 2.523665520064312, Math.toRadians(358.47773335618814)))
                .build();

        Trajectory traj9 = driveBase.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(41.108323, 2.523665520064312, 5.85648))
                .build();

        Trajectory traj10 = driveBase.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(56.321, 9.896, 6.05))
                .build();

        Trajectory traj11 = driveBase.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(98.42439, 17.0, Math.toRadians(90)))
                .build();

        Trajectory traj12 = driveBase.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(94.42439, 17.0, Math.toRadians(90)))
                .build();

        wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

        waitForStart();

        wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[0]);

        shooter.SetShooter(Constants.SHOOTER_VELOCITY);
        intake.ReleaseWalls();
        intake.SetWallPosition(0.5, 0.3);

        while (!isStopRequested())
        {
            telemetry.addData("Current Trajectory", "Trajectory 1");
            telemetry.update();
            driveBase.followTrajectory(traj1);

            intake.SetWallPosition(Constants.LEFT_WALL_POS_IN, Constants.RIGHT_WALL_POS_IN);

            telemetry.addData("Current Trajectory", "Trajectory 2");
            telemetry.update();
            driveBase.followTrajectory(traj2);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(300);

                shooter.Unkick();

                sleep(300);
            }

            shooter.Kick();

            sleep(300);

            shooter.Unkick();

            telemetry.addData("Current Trajectory", "Trajectory 3");
            telemetry.update();
            driveBase.followTrajectory(traj3);

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            timer.reset();

            while (timer.seconds() < 0.50 && opModeIsActive())
            {}

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.25 && opModeIsActive())
            {}

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[2]);

            telemetry.addData("Current Trajectory", "Trajectory 4");
            telemetry.update();
            driveBase.followTrajectory(traj4);

            telemetry.addData("Current Trajectory", "Trajectory 5");
            telemetry.update();
            driveBase.followTrajectory(traj5);

            telemetry.addData("Current Trajectory", "Trajectory 6");
            telemetry.update();
            driveBase.followTrajectory(traj6);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_CLOSE_POS);

            timer.reset();

            while (timer.seconds() < 0.25 && opModeIsActive())
            {}

            wobbleGoal.GoToWobbleGoalPosition(Constants.WOBBLE_GOAL_POSITION_VALUES[1]);

            telemetry.addData("Current Trajectory", "Trajectory 7");
            telemetry.update();
            driveBase.followTrajectory(traj7);

            intake.SetIntake(1, 0);

            telemetry.addData("Current Trajectory", "Trajectory 8");
            telemetry.update();
            //driveBase.followTrajectory(traj8);
            driveBase.followTrajectory(traj9);

            intake.SetIntake(0,0);
            shooter.SetShooter(Constants.SHOOTER_VELOCITY*1.05);

            telemetry.addData("Current Trajectory", "Trajectory 10");
            telemetry.update();
            driveBase.followTrajectory(traj10);

            for(int i = 0; i < 2; i++)
            {
                shooter.Kick();

                sleep(300);

                shooter.Unkick();

                sleep(300);
            }

            shooter.Kick();

            sleep(300);

            shooter.Unkick();

            intake.SetWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

            telemetry.addData("Current Trajectory", "Trajectory 11");
            telemetry.update();
            driveBase.followTrajectory(traj11);

            wobbleGoal.GoToPosWobbleGoalManipulatorHandler(Constants.WOBBLE_GOAL_MANIPULATOR_SERVO_OPEN_POS);

            timer.reset();

            while (timer.seconds() < 0.5 && opModeIsActive())
            {}

            telemetry.addData("Current Trajectory", "Trajectory 12");
            telemetry.update();
            driveBase.followTrajectory(traj12);

            PoseStorage.currentPose = new Pose2d(driveBase.getPoseEstimate().getX(),driveBase.getPoseEstimate().getY(),
                    driveBase.getPoseEstimate().getHeading());

            requestOpModeStop();
        }
    }
}
