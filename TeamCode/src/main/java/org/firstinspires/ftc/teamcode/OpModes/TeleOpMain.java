package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.WobbleGoal;
import org.firstinspires.ftc.teamcode.DriveBase.drive.DriveBase;

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

        telemetry.addLine("System Initialization Complete");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        if(isStopRequested()) return;

        shooter.SetShooter();

        while (!isStopRequested() && opModeIsActive())
        {
            Pose2d poseEstimate = driveBase.getPoseEstimate();

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

            driveBase.update();

            shooter.Shoot(gamepad1.a);
            intake.SetIntake(gamepad1.right_trigger, gamepad1.left_trigger);
            wobbleGoal.MoveWobbleGoalPosition(gamepad2.dpad_left, gamepad2.dpad_up, gamepad2.dpad_right);
            wobbleGoal.WobbleGoalManipulatorHandler(gamepad2.x, gamepad2.b);
        }
    }
}
