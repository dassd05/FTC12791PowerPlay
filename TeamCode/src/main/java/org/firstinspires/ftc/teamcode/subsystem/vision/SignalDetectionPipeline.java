package org.firstinspires.ftc.teamcode.subsystem.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * OpenCV pipeline for detecting our custom signal sleeve. Uses black as position 1,
 * white as position 2, and yellow as position 3.
 */
@Config
public class SignalDetectionPipeline extends OpenCvPipeline {

    public enum ParkPosition {
        LEFT, MIDDLE, RIGHT
    }

    /**
     * The region of interest; a rectangle on the frame from which we are deriving
     * our position from.
     */
    public static int roiX = 490;
    public static int roiY = 520;
    public static int roiW = 30;
    public static int roiH = 60;
//    Rect regionOfInterest = new Rect(new double[] { 380, 230, 100, 200 }); // x, y, w, h
    private Mat region;

    // black is roughly 180-200 and white is roughly 450-500
    public static int blackWhiteVal = 420;

    /**
     * The current position guessed based on the camera input.
     */
    public volatile ParkPosition position;
    /**
     * A Scalar color of the average the pixels in the region of interest.
     */
    public volatile Scalar average;

    @Override
    public void init(Mat mat) {
        region = mat.submat(new Rect(roiX, roiY, roiW, roiH));
    }

    @Override
    public Mat processFrame(Mat input) {
        average = Core.mean(region);

        Imgproc.rectangle(input, new Rect(roiX, roiY, roiW, roiH), invertColor(average), 5);

        position = ParkPosition.MIDDLE;
        if (average.val[0] + average.val[1] + average.val[2] < blackWhiteVal) position = ParkPosition.LEFT;
        if (average.val[0] - average.val[2] > 35 && average.val[1] - average.val[2] > 35) position = ParkPosition.RIGHT;

        return input;
    }

    /**
     * Takes a Scalar color and inverts it. So {@code 255 - val} for each color.
     * @param color The color to be inverted.
     * @return The inverted color.
     */
    public static Scalar invertColor(Scalar color) {
        return new Scalar(255 - color.val[0], 255 - color.val[1], 255 - color.val[2], color.val[3]);
    }
}
