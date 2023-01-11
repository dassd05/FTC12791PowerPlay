package org.firstinspires.ftc.teamcode.subsystem.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionDetectionPipeline extends OpenCvPipeline {

    public volatile double angleOff;

    private List<MatOfPoint> contours;
    private Mat hierarchy;

    public double getAngleOff() {
        return angleOff;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat copy = new Mat(input.size(), input.type());
        input.copyTo(copy);
        copy = inRange(sharpen(copy));
        input = drawBoxes(input, findObj(copy));
        return input;
    }

    public Mat inRange(Mat frame) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2HSV);
        Core.inRange(frame, new Scalar(20, 200, 120), new Scalar(30, 255, 255), frame);
        return frame;
    }

    public Mat edgeDetect(Mat frame) {
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        kernel.put(0, 0, -.25f, 0f, .25f, -.5f, 0f, .5f, -.25f, 0f, .25f);
        Imgproc.filter2D(frame, frame, -1, kernel, new Point(-1, -1), 127);
        return frame;
    }

    public Mat sharpen(Mat frame) {
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        kernel.put(0, 0, -.25f, -.5f, -.25f, -.5f, 4f, -.5f, -.25f, -.5f, -.25f);
        Imgproc.filter2D(frame, frame, -1, kernel);
        return frame;
    }

    public Mat vertical(Mat frame) {
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        kernel.put(0, 0, -.25f, -.5f, -.25f, -.5f, 4f, -.5f, -.25f, -.5f, -.25f);
        Imgproc.filter2D(frame, frame, -1, kernel);
        return frame;
    }

    public List<MatOfPoint> findObj(Mat frame) {
        contours = new ArrayList<>();
        hierarchy = new Mat();
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    public Mat drawBoxes(Mat frame, List<MatOfPoint> contours) {
        for (MatOfPoint contour : contours) {
            Rect rectangle = Imgproc.boundingRect(contour);
            if (rectangle.width > 5 && rectangle.width < 150 && rectangle.height > 75)
                Imgproc.rectangle(frame, new Point(rectangle.x, rectangle.y), new Point(rectangle.x + rectangle.width, rectangle.y + rectangle.height), new Scalar(36, 255, 12), 1);
        }
        return frame;
    }
}
