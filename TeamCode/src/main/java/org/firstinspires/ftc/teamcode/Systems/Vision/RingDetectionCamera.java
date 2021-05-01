package org.firstinspires.ftc.teamcode.Systems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.Vision.Pipelines.RingDeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RingDetectionCamera
{

    WebcamName webcamName;
    int cameraMonitorViewId;
    OpenCvCamera camera;

    RingDeterminationPipeline pipeline;

    public RingDetectionCamera(HardwareMap hardwareMap)
    {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, Constants.RING_DETECTION_CAMERA_NAME);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(() -> {
            camera.startStreaming(Constants.CAMERA_RESOLUTION_WIDTH, Constants.CAMERA_RESOLUTION_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            FtcDashboard.getInstance().startCameraStream(camera, 0);
        });
    }

    public RingDeterminationPipeline.RingPosition getValue()
    {
        return pipeline.position;
    }

    public double getAnalysis()
    {
        return pipeline.getAnalysis();
    }
}
