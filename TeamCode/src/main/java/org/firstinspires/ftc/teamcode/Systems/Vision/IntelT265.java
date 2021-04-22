package org.firstinspires.ftc.teamcode.Systems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class IntelT265 {

    T265Camera iT265Camera;

    public IntelT265(HardwareMap hardwareMap)
    {

        iT265Camera = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        iT265Camera.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin

         // Call this when you're ready to get camera updates
        iT265Camera.start();
    }

    public T265Camera.CameraUpdate getPosition()
    {
       return iT265Camera.getLastReceivedCameraUpdate();
    }
}
