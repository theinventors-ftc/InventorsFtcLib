package org.inventors.ftc.robotbase.hardware;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.opencvpipelines.TeamPropDetectionPipeline;
import org.inventors.ftc.robotbase.RobotMapInterface;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    private final OpenCvWebcam webcam;
    protected TeamPropDetectionPipeline teamPropDetectionPipeline;

    private Telemetry telemetry;

    public Camera(RobotMapInterface robotMap, FtcDashboard dashboard) {
        this.telemetry = robotMap.getTelemetry();

        this.webcam = robotMap.getCamera();

        dashboard.startCameraStream(webcam, 0);

        webcam.setPipeline(teamPropDetectionPipeline);

        // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
}
