 package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
