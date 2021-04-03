package org.firstinspires.ftc.teamcode.Vision;

//----------------------------------------------------------------------------
//
//  $Workfile: JimsTest02.java$
//
//  $Revision: X$
//
//  Project:    FTC 2021
//
//                            Copyright (c) 2021
//                              Stealth Robotics
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  Imports
//----------------------------------------------------------------------------
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Vision.RingWebCam;
import org.firstinspires.ftc.teamcode.Vision.RingLocation;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: JimsTest02
//
//Purpose:
//  This is an example of a robot using the Webcam to detirmine the number of rings,
//  while doing something else.
//
//----------------------------------------------------------------------------
@TeleOp(name = "JimsTest02", group = "Concept")
public class JimsTest02 extends LinearOpMode {
    //----------------------------------------------------------------------------
    //  Class Atributes
    //----------------------------------------------------------------------------
    private RingWebCam ringWebCam;

    //----------------------------------------------------------------------------
    //  Purpose:
    //   Runs the op mode
    //
    //  Notes:
    //
    //
    //----------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        //  Builds a ringlocation that stores the found stack
        RingLocation lastReconizedLocation = new RingLocation();

        ringWebCam = new RingWebCam();
        ringWebCam.setHardwareMap(hardwareMap);
        ringWebCam.init();


        waitForStart();
        while(opModeIsActive()) {
            RingLocation location = ringWebCam.findStack(telemetry);
            if(0 != location.mDetected)
            {
                lastReconizedLocation = location;
            }
            telemetry.addData("stack", String.format("%d", lastReconizedLocation.mDetected));
            telemetry.update();
        }

        ringWebCam.shutdown();
    }
}
