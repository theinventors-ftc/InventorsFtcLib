package org.inventors.ftc.robotbase.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.inventors.ftc.opencvpipelines.AprilTagDetectionPipeline;
import org.inventors.ftc.opencvpipelines.TeamPropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public class Camera {
    private final OpenCvWebcam webcam;
//    protected AprilTagDetectionPipeline april_tag_pipeline;
    protected TeamPropDetectionPipeline teamPropDetectionPipeline;

    private Telemetry telemetry;

    public Camera(HardwareMap hardwareMap, FtcDashboard dashboard, Telemetry telemetry, TeamPropDetectionPipeline.Alliance alliance) {
        this.telemetry = telemetry;

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "webcam"), cameraMonitorViewId);

        dashboard.startCameraStream(webcam, 0);

//        april_tag_pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry, alliance, 30, 5);
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

    public int getTeamPropPos() {
        return teamPropDetectionPipeline.getPropPosition();
    }
}
