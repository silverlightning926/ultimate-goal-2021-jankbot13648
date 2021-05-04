package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.WobbleGoal;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;

import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.TRACK_WIDTH;

@TeleOp(name = "TeleOp - Main")
public class TeleOpMain extends LinearOpMode {

    Telemetry dashboardTelemetry;

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;

    double autoAimOffset;

    @Override
    public void runOpMode() {

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        driveBase = new DriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);

        autoAimOffset = Constants.AUTO_AIM_OFFSET;

        driveBase.setPoseEstimate(PoseStorage.currentPose);

        telemetry.addLine("System Initialization Complete");
        telemetry.update();
        
        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        if(isStopRequested()) return;

        shooter.unKick();
        shooter.setShooter(Constants.TELEOP_SHOOTER_SPEED);

        while (!isStopRequested() && opModeIsActive())
        {
            if(gamepad1.a)
            {
                AutoAimShoot();
            }

            else if (gamepad1.b)
            {
                Shoot();
            }

            else
            {
                Pose2d poseEstimate = driveBase.getPoseEstimate();

                Vector2d input;

                if(!gamepad1.left_bumper)
                {
                    input = new Vector2d(
                            -gamepad1.left_stick_x,
                            gamepad1.left_stick_y
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
                    input = new Vector2d(
                            -gamepad1.left_stick_x/2,
                            gamepad1.left_stick_y/2
                    ).rotated(-poseEstimate.getHeading());

                    driveBase.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad1.right_stick_x * 0.2
                            )
                    );
                }


            }

            driveBase.update();

            if(gamepad1.dpad_up)
            {
                /**
                 * @todo Align Powershots Automatically
                 * @body Use odometry to align with the first powershot and start the sequence automatically
                 */

                PowerShotTrajectory();
            }

            if(gamepad1.dpad_down)
            {
                driveBase.setPoseEstimate(
                        new Pose2d(
                                59.597240135489834,
                                -41.06490535746197,
                                0
                        ));
            }

            if(gamepad2.left_bumper)
            {
                autoAimOffset += 0.0625;
            }

            if(gamepad2.right_bumper)
            {
                autoAimOffset -= 0.0625;
            }

            intake.setIntake(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.right_bumper);
            wobbleGoal.moveWobbleGoalPosition(gamepad2.dpad_up, gamepad2.dpad_right, gamepad2.dpad_down);
            wobbleGoal.moveWobbleGoalManipulator(gamepad2.x, gamepad2.b);
        }
    }

    private void AutoAimShoot()
    {
        double error_X = Constants.GOAL_X_COORD - driveBase.getPoseEstimate().getX();
        double error_Y = Constants.GOAL_Y_COORD - driveBase.getPoseEstimate().getY();

        double aimAngle = Math.toDegrees(Math.atan2(error_Y, error_X)) + autoAimOffset;

        telemetry.addData("Error X", error_X);
        telemetry.addData("Error Y", error_Y);
        telemetry.addData("Turning To", aimAngle);
        telemetry.update();

        driveBase.turnToAutoAim(Math.toRadians(aimAngle));

        Shoot();
    }

    private void Shoot() {

        intake.setWallPosDown();

        for (int i = 0; i < 2; i++) {
            shooter.kick();

            sleep(Constants.SHOOTER_DELAY);

            shooter.unKick();

            sleep(Constants.DROP_DELAY);
        }

        shooter.kick();

        sleep(Constants.SHOOTER_DELAY);

        shooter.unKick();
    }

    private void PowerShotTrajectory() {

        driveBase.setPoseEstimate(new Pose2d(
                driveBase.getPoseEstimate().getX(),
                driveBase.getPoseEstimate().getY(),
                0
        ));

        autoAimOffset = Constants.AUTO_AIM_OFFSET;

        shooter.setShooter(Constants.POWER_SHOT_VELOCITY);

        Trajectory traj1 = driveBase.trajectoryBuilder(driveBase.getPoseEstimate())
                .lineToConstantHeading(
                        new Vector2d(
                                driveBase.getPoseEstimate().getX(),
                                driveBase.getPoseEstimate().getY()+11.375),

                        DriveBase.getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH),
                        DriveBase.getAccelerationConstraint(30)
                )

                .build();

        driveBase.followTrajectory(traj1);

        intake.setWallPosition(Constants.LEFT_WALL_POS_OUT, Constants.RIGHT_WALL_POS_IN);

        shooter.kick();

        sleep(Constants.SHOOTER_DELAY);

        shooter.unKick();

        sleep(100);

        driveBase.turn(Math.toRadians(Constants.POWER_SHOT_TURN-0.25), Math.toRadians(60), Math.toRadians(20));

        shooter.kick();

        sleep(Constants.SHOOTER_DELAY);

        shooter.unKick();

        sleep(100);

        driveBase.turn(Math.toRadians(Constants.POWER_SHOT_TURN - 0.0625), Math.toRadians(60), Math.toRadians(20));

        shooter.kick();

        sleep(Constants.SHOOTER_DELAY);

        shooter.unKick();

        sleep(100);

        shooter.setShooter(Constants.TELEOP_SHOOTER_SPEED);
    }
}
