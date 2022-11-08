package org.firstinspires.ftc.teamcode.robot.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.Collections;

public class SignalDetectionPipeline extends OpenCvPipeline {

    public enum ParkPosition {
        LEFT, MIDDLE, RIGHT
    }

    Rect regionOfInterest = new Rect(new double[] { 0, 0, 0, 0 }); // x, y, w, h

    private Mat region;

    public volatile ParkPosition position;


    @Override
    public void init(Mat firstFrame) {
        region = firstFrame.submat(regionOfInterest);
    }

    @Override
    public Mat processFrame(Mat input) {
        Scalar average = Core.mean(region);

        Imgproc.rectangle(input, regionOfInterest, invertColor(average), 5);

        int greatest = 0;
        if (average.val[1] > average.val[0]) greatest = 1;
        if (average.val[2] > average.val[greatest]) greatest = 2;

        if (greatest == 0) position = ParkPosition.LEFT;
        if (greatest == 1) position = ParkPosition.MIDDLE;
        if (greatest == 2) position = ParkPosition.RIGHT;

        Scalar detected = Scalar.all(0);
        detected.val[greatest] = 255;
        Imgproc.rectangle(input, regionOfInterest, detected, 2);

        return input;
    }

    public static Scalar invertColor(Scalar color) {
        return new Scalar(255 - color.val[0], 255 - color.val[1], 255 - color.val[2], color.val[3]);
    }
}
