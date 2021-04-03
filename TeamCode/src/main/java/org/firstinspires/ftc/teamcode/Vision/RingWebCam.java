package org.firstinspires.ftc.teamcode.Vision;

//----------------------------------------------------------------------------
//
//  $Workfile: RingLocation.java$
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.*;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Vision.RingLocation;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: RingWebCam
//
//Purpose:
//  This is the class handles the web cam and tensor flow.
//
//----------------------------------------------------------------------------
public class RingWebCam {
    //----------------------------------------------------------------------------
    //  Class Constants
    //----------------------------------------------------------------------------
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AdcmR0H/////AAABmX5FCuasR0WJvaW+WfKBZZY4Mkmr39mE5zeNXTSqAfbDNYf8W53AS1tK2Fgrwh1kN/CVmyZnbLlKxrnI9kDObs2/9mddaFBkwb7TCKrCo4Cy/kqF85YWtWYekU3EqHzuMNvw1OonPzQJ7kjgBhszai3nQKg5bN2dd8NBPFpScozkSlvIUrWFvbj20K1K7kQ3JjgMreeSNldB12C+ZeeeUi9IVDBqfkcBOszPh0HvSMBGX3IkIkac56/UcTFbaa/GNWTwZtrVshjnypXm15Y1d62Ehg9G8wNHFMwhvzHqfGvLA++K7x/dCtB+iaPE7WNxom4RaE+UYJ8kDjbpjrbtiXPuqUN9+SbCwQK6IN86vAer";

    //----------------------------------------------------------------------------
    //  Class Atributes
    //----------------------------------------------------------------------------
    private VuforiaLocalizer vuforia;
    private HardwareMap hardwareMap;
    private TFObjectDetector tfod;

    //----------------------------------------------------------------------------
    //  Purpose:
    //   Loads our hardware map from the main program
    //
    //  Notes:
    //      It needs to be called first or bad things happen!!!
    //
    //----------------------------------------------------------------------------
    public void setHardwareMap(HardwareMap map) {
        hardwareMap = map;
    }

    //----------------------------------------------------------------------------
    //  Purpose:
    //   This finds the web cam and gets it set up
    //
    //  Notes:
    //      It needs to be called before findStack
    //
    //----------------------------------------------------------------------------
    public void init() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        initTfod();
    }

    //----------------------------------------------------------------------------
    //  Purpose:
    //   This gets the tensorflow running
    //
    //  Notes:
    //
    //
    //----------------------------------------------------------------------------
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
    }

    //----------------------------------------------------------------------------
    //  Purpose:
    //   This finds the stack
    //
    //  Notes:
    //
    //
    //----------------------------------------------------------------------------
    public RingLocation findStack(Telemetry telemetry) {
        RingLocation location = new RingLocation();

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    location.mTop = recognition.getTop();
                    location.mLeft = recognition.getLeft();
                    location.mRight = recognition.getRight();
                    location.mBottom = recognition.getBottom();

                    if(recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                    {
                        location.mDetected = RingLocation.TARGET_ONE_RING;
                    }

                    if(recognition.getLabel().equals(LABEL_FIRST_ELEMENT))
                    {
                        location.mDetected = RingLocation.TARGET_FOUR_RINGS;
                    }
                }
            }
        }
        return location;
    }

    //----------------------------------------------------------------------------
    //  Purpose:
    //   Shuts down the tensor flow
    //
    //  Notes:
    //
    //
    //----------------------------------------------------------------------------
    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}