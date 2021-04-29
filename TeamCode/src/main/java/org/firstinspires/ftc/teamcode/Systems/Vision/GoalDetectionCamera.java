package org.firstinspires.ftc.teamcode.Systems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Systems.Vision.Pipelines.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.Systems.Vision.Pipelines.UGAdvancedHighGoalPipeline;
import org.firstinspires.ftc.teamcode.Systems.Vision.Pipelines.UGBasicHighGoalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class GoalDetectionCamera
{

    WebcamName webcamName;
    int cameraMonitorViewId;
    OpenCvCamera camera;

    public UGBasicHighGoalPipeline pipeline;

    public GoalDetectionCamera(HardwareMap hardwareMap)
    {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, Constants.GOAL_DETECTION_CAMERA_NAME);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new UGAdvancedHighGoalPipeline(68.5, 16.5);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(() -> {
            camera.startStreaming(Constants.CAMERA_RESOLUTION_WIDTH, Constants.CAMERA_RESOLUTION_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            FtcDashboard.getInstance().startCameraStream(camera, 0);
        });
    }
}
