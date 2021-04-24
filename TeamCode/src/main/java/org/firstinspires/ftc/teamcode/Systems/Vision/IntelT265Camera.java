package org.firstinspires.ftc.teamcode.Systems.Vision;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.Constants;

public class IntelT265Camera {

    T265Camera t265Camera = null;

    T265Camera.CameraUpdate lastCameraUpdate;

    public IntelT265Camera(HardwareMap hardwareMap)
    {
        if(t265Camera == null)
            t265Camera = new T265Camera(Constants.CAMERA_TO_ROBOT, Constants.ENCODER_MEASUREMENT_COVARIANCE, hardwareMap.appContext);

        t265Camera.start();
    }

    public boolean isStarted()
    {
        return t265Camera.isStarted();
    }

    public void stop()
    {
        t265Camera.stop();
    }

    public void update()
    {
        lastCameraUpdate = t265Camera.getLastReceivedCameraUpdate();
    }

    public T265Camera.CameraUpdate getLastCameraUpdate()
    {
        return lastCameraUpdate;
    }
}
