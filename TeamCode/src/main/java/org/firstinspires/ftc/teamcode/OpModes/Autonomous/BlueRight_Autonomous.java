package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.DriveBase.drive.DriveBase;
import org.firstinspires.ftc.teamcode.DriveBase.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.WobbleGoal;

public class BlueRight_Autonomous extends LinearOpMode {

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

        //driveBase.setPoseEstimate(Constants.STARTING_POSE);

        waitForStart();

        while (!isStopRequested())
        {
            // DO AUTONOMOUS STUFF HERE

            requestOpModeStop();
        }

        PoseStorage.currentPose = driveBase.getPoseEstimate();
    }
}
