package org.inventors.ftc.opencvpipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetectionPipeline extends OpenCvPipeline {
    public enum Alliance {
        RED,
        BLUE
    }

    public Alliance allianceType = Alliance.RED;

    public double threshold = 40;

    Mat blurred = new Mat();
    Mat redChannel = new Mat();
    Mat greenChannel = new Mat();
    Mat blueChannel = new Mat();

    // ------------------------------------ For Red Detection ----------------------------------- //
    Mat redGreenDif = new Mat();
    Mat redBlueDif = new Mat();
    Mat redGreenThresh = new Mat();
    Mat redBlueThresh = new Mat();

    // ----------------------------------- For Blue Detection ----------------------------------- //
    Mat blueGreenDif = new Mat();
    Mat blueRedDif = new Mat();
    Mat blueGreenThresh = new Mat();
    Mat blueRedThresh = new Mat();

    // ------------------------------------------------------------------------------------------ //

    Mat thresh = new Mat();

    Mat leftROI = new Mat();
    Mat centerROI = new Mat();
    Mat rightROI = new Mat();

    Rect leftRect = new Rect(40, 420, 400, 290);
    Rect centerRect = new Rect(520, 350, 320, 319);
    Rect rightRect = new Rect(900, 310, 300, 400);

    double leftMean = 0.0, centerMean = 0.0, rightMean = 0.0;

    Telemetry telemetry;

    double blurAmount = 0.0;

    public TeamPropDetectionPipeline(Telemetry telemetry, Alliance allianceType, double threshhold,
                                     double blurAmount) {
        this.telemetry = telemetry;
        this.allianceType = allianceType;
        this.threshold = threshhold;
        this.blurAmount = blurAmount;
    }

    public TeamPropDetectionPipeline(Telemetry telemetry, Alliance allianceType, double threshhold) {
        this.telemetry = telemetry;
        this.allianceType = allianceType;
        this.threshold = threshhold;
        this.blurAmount = 6;
    }

    public TeamPropDetectionPipeline(Telemetry telemetry, Alliance allianceType) {
        this.telemetry = telemetry;
        this.allianceType = allianceType;
        this.threshold = 60;
        this.blurAmount = 6;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Blur a bit the image for smoother edges
        Imgproc.blur(input, blurred, new Size(5, 5));

        // Split the channels so they are compared later
        Core.extractChannel(blurred, redChannel, 0);
        Core.extractChannel(blurred, greenChannel, 1);
        Core.extractChannel(blurred, blueChannel, 2);

        if (allianceType == Alliance.RED) {
            Core.subtract(redChannel, greenChannel, redGreenDif);
            Core.subtract(redChannel, blueChannel, redBlueDif);

            // Make Binary Images
            Core.compare(redGreenDif, new Scalar(threshold, threshold), redGreenThresh, Core.CMP_GT);
            Core.compare(redBlueDif, new Scalar(threshold, threshold), redBlueThresh, Core.CMP_GT);

            // Combine Binary Images
            Core.bitwise_and(redGreenThresh, redBlueThresh, thresh);
        } else if (allianceType == Alliance.BLUE) {
            Core.subtract(blueChannel, greenChannel, blueGreenDif);
            Core.subtract(blueChannel, redChannel, blueRedDif);

            // Make Binary Images
            Core.compare(blueGreenDif, new Scalar(threshold, threshold), blueGreenThresh, Core.CMP_GT);
            Core.compare(blueRedDif, new Scalar(threshold, threshold), blueRedThresh, Core.CMP_GT);

            // Combine Binary Images
            Core.bitwise_and(blueGreenThresh, blueRedThresh, thresh);
        }

        // Split the Image into ROIs
        leftROI = new Mat(thresh, leftRect);
        centerROI = new Mat(thresh, centerRect);
        rightROI = new Mat(thresh, rightRect);

        // Calculate Means
        leftMean = Core.mean(leftROI).val[0];
        centerMean = Core.mean(centerROI).val[0];
        rightMean = Core.mean(rightROI).val[0];

        // Print Data (only for experimenting)
        telemetry.addData("Left Mean: ", leftMean);
        telemetry.addData("Center Mean: ", centerMean);
        telemetry.addData("Right Mean: ", rightMean);
        telemetry.addData("Position: ", getPropPosition());

        return thresh;
    }

    public void setThreshold(int newThreshold) {
        this.threshold = newThreshold;
    }

    // This function returns the Position of the Team Prop. 0 -> Left, 1 -> Center,
    // 2 -> Right
    public int getPropPosition() {
        double[] means = { leftMean, centerMean, rightMean };

        int indexOfLargest = 0;
        double max = means[0];

        for (int i = 1; i < means.length; i++) {
            if (means[i] > max) {
                max = means[i];
                indexOfLargest = i;
            }
        }

        return indexOfLargest;
    }
}
