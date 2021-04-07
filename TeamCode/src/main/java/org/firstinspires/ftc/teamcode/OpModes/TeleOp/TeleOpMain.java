package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.WobbleGoal;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;

@TeleOp(name = "TeleOp - Main")
public class TeleOpMain extends LinearOpMode {

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;

    @Override
    public void runOpMode() throws InterruptedException {

        driveBase = new DriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);

        driveBase.setPoseEstimate(PoseStorage.currentPose);

        telemetry.addLine("System Initialization Complete");
        telemetry.update();
        
        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        shooter.setShooter(Constants.SHOOTER_VELOCITY);

        if(isStopRequested()) return;

        shooter.unKick();

        while (!isStopRequested() && opModeIsActive())
        {
            if(gamepad1.a)
            {
                intake.setWallPosDown();

                Shoot();
            }

            else
            {
                Pose2d poseEstimate = driveBase.getPoseEstimate();

                if(!gamepad1.right_bumper)
                {
                    Vector2d input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(-poseEstimate.getHeading());

                    driveBase.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad1.right_stick_x
                            )
                    );
                }

                else {
                    Vector2d input = new Vector2d(
                            -gamepad1.left_stick_y/2,
                            -gamepad1.left_stick_x/2
                    ).rotated(-poseEstimate.getHeading());

                    driveBase.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad1.right_stick_x/2
                            )
                    );
                }

                driveBase.update();
            }


            if(gamepad1.dpad_up)
            {
                /**
                 * @todo Align Powershots Automatically
                 * @body Use odometry to align with the first powershot and start the sequence automatically
                 */

                shooter.setShooter(Constants.POWER_SHOT_VELOCITY);

                Trajectory powerShot_traj1 = driveBase.trajectoryBuilder(new Pose2d(
                        driveBase.getPoseEstimate().getX(),
                        driveBase.getPoseEstimate().getY(),
                        driveBase.getPoseEstimate().getHeading()
                ))
                        .strafeLeft(6.5)
                        .build();

                Trajectory powerShot_traj2 = driveBase.trajectoryBuilder(powerShot_traj1.end())
                        .strafeLeft(7.5)
                        .build();

                Trajectory powerShot_traj3 = driveBase.trajectoryBuilder(powerShot_traj2.end())
                        .strafeLeft(7.5)
                        .build();

                driveBase.followTrajectory(powerShot_traj1);

                shooter.kick();

                sleep(200);

                shooter.unKick();

                driveBase.followTrajectory(powerShot_traj2);

                shooter.kick();

                sleep(200);

                shooter.unKick();

                driveBase.followTrajectory(powerShot_traj3);

                shooter.kick();

                sleep(200);

                shooter.unKick();

                shooter.setShooter(Constants.SHOOTER_VELOCITY);

            }

            intake.setIntake(gamepad1.right_trigger, gamepad1.left_trigger);
            wobbleGoal.moveWobbleGoalPosition(gamepad2.dpad_left, gamepad2.dpad_right);
            wobbleGoal.moveWobbleGoalManipulator(gamepad2.x, gamepad2.b);
        }
    }

    private void Shoot() {

        /**
         * @todo Add Auto Aim
         * @body Turn the drive-base to correct heading automatically before shooting
         */

        for (int i = 0; i < 2; i++) {
            shooter.kick();

            sleep(250);

            shooter.unKick();

            sleep(250);
        }

        shooter.kick();

        sleep(250);

        shooter.unKick();
    }
}
