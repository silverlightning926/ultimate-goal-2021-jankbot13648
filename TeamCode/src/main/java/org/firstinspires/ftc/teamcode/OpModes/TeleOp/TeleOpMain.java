package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Vision.GoalDetectionCamera;
import org.firstinspires.ftc.teamcode.Systems.WobbleGoal;
import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveBase;

@TeleOp(name = "TeleOp - Main")
public class TeleOpMain extends LinearOpMode {

    Telemetry dashboardTelemetry;

    DriveBase driveBase;
    Shooter shooter;
    Intake intake;
    WobbleGoal wobbleGoal;
    GoalDetectionCamera goalDetectionCamera;

    PIDFController autoAimController;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        driveBase = new DriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        wobbleGoal = new WobbleGoal(hardwareMap);
        goalDetectionCamera = new GoalDetectionCamera(hardwareMap);

        autoAimController = new PIDFController(Constants.AUTO_AIM_COEFFICIENTS);
        autoAimController.setOutputBounds(-Constants.AUTO_AIM_SPEED, Constants.AUTO_AIM_SPEED);

        driveBase.setPoseEstimate(PoseStorage.currentPose);

        telemetry.addLine("System Initialization Complete");
        telemetry.update();
        
        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        if(isStopRequested()) return;

        shooter.unKick();
        //shooter.setShooter(Constants.SHOOTER_VELOCITY);

        while (!isStopRequested() && opModeIsActive())
        {
            autoAimController.update(goalDetectionCamera.pipeline.getCenterofRect(goalDetectionCamera.pipeline.getRedRect()).x);

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
                                    -gamepad1.right_stick_x * 0.3
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

            intake.setIntake(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.right_bumper);
            wobbleGoal.moveWobbleGoalPosition(gamepad2.dpad_up, gamepad2.dpad_right, gamepad2.dpad_down);
            wobbleGoal.moveWobbleGoalManipulator(gamepad2.x, gamepad2.b);
        }

        FtcDashboard.getInstance().stopCameraStream();
    }

    private void AutoAimShoot()
    {
        autoAimController.setTargetPosition(550);

        while (Math.abs(autoAimController.getLastError()) > Constants.AUTO_AIM_ALLOWABLE_ERROR)
        {
            driveBase.update();

            double update = autoAimController.update(goalDetectionCamera.pipeline.getCenterofRect(goalDetectionCamera.pipeline.getRedRect()).x);

            driveBase.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0,
                            update
                    )
            );

            dashboardTelemetry.addData("Update", update);
            dashboardTelemetry.addData("Position", goalDetectionCamera.pipeline.getCenterofRect(goalDetectionCamera.pipeline.getRedRect()).x);
            dashboardTelemetry.addData("Target", 550);
            dashboardTelemetry.update();
        }

        driveBase.setWeightedDrivePower(
                new Pose2d()
        );

        Shoot();
    }

    private void Shoot() {
        for (int i = 0; i < 2; i++) {
            shooter.kick();

            sleep(Constants.SHOOTER_DELAY);

            shooter.unKick();

            sleep(Constants.DROP_DELAY);
        }

        intake.setWallPosDown();

        shooter.kick();

        sleep(Constants.SHOOTER_DELAY);

        shooter.unKick();
    }

    private void PowerShotTrajectory() {

        shooter.setShooter(Constants.POWER_SHOT_VELOCITY);

        driveBase.turn(-Math.toRadians(Constants.POWER_SHOT_TURN_OFFSET), Math.toRadians(60), Math.toRadians(30));

        for (int i = 0; i < 3; i++)
        {
            driveBase.turn(-Math.toRadians(Constants.POWER_SHOT_TURN), Math.toRadians(60), Math.toRadians(30));

            shooter.kick();

            sleep(200);

            shooter.unKick();
        }

        shooter.setShooter(Constants.SHOOTER_VELOCITY);
    }
}
