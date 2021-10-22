 package org.firstinspires.ftc.teamcode.OpModes;
        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.Constants;

        import org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.MecanumDrive;

        import static org.firstinspires.ftc.teamcode.Systems.DriveBase.drive.DriveConstants.*;

        import java.util.List;

 @TeleOp(name = "TeleOp - Main")
public class TeleOpMain extends LinearOpMode {
     
     @Override
     public void runOpMode() {

         waitForStart();

         if (isStopRequested()) return;

         while (!isStopRequested() && opModeIsActive())
         {

         }
     }
 }
