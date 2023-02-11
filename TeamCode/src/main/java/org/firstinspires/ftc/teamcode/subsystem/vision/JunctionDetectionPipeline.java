package org.firstinspires.ftc.teamcode.subsystem.vision;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystem.C270Info;
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
import java.util.Locale;

@Config
public class JunctionDetectionPipeline extends OpenCvPipeline {
    public static boolean maskedView = false;
    public static int yellowLowH = 95;//20;
    public static int yellowLowS = 175;
    public static int yellowLowV = 120;
    public static int yellowHighH = 100;//30;
    public static int yellowHighS = 255;
    public static int yellowHighV = 255;

    private final Object junctionLock = new Object();

    private List<Target> junctions;

    public List<Target> getJunctions() {
        synchronized (junctionLock) {
            return new ArrayList<>(junctions);
        }
    }
    @Nullable public Target getClosestJunction() {
        synchronized (junctionLock) {
            if (junctions.size() == 0) return null;
            Target largest = junctions.get(0);
            for (Target junction : junctions) {
                if (junction.rect.width > largest.rect.width) largest = junction;
            }
            return largest;
        }
    }
    @Nullable public Target getMostCenteredJunction() {
        synchronized (junctionLock) {
            if (junctions.size() == 0) return null;
            Target closest = junctions.get(0);
            for (Target junction : junctions) {
                if (Math.abs(junction.offset) < Math.abs(closest.offset)) closest = junction;
            }
            return closest;
        }
    }
    // todo get junction by height

    @Override
    public Mat processFrame(Mat input) {
        Mat copy = new Mat(input.size(), CvType.CV_8UC1);
        inRange(input, copy);
        List<MatOfPoint> contours = findObj(copy);
        List<Rect> rects = drawBoxes(input, contours);
        List<Double> offsets = findOffsets(input, rects);

        synchronized (junctionLock) {
            junctions = new ArrayList<>();
            for (int i = 0; i < rects.size(); i++)
                junctions.add(new Target(contours.get(i), rects.get(i), offsets.get(i)));
        }

        return maskedView ? copy : input;
    }

    public void inRange(Mat frame, Mat dest) {
        Imgproc.cvtColor(frame, dest, Imgproc.COLOR_BGR2HSV);
        Core.inRange(dest, new Scalar(yellowLowH, yellowLowS, yellowLowV), new Scalar(yellowHighH, yellowHighS, yellowHighV), dest);
    }

    public void edgeDetect(Mat frame, Mat dest) {
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        kernel.put(0, 0, -.25f, 0f, .25f, -.5f, 0f, .5f, -.25f, 0f, .25f);
        Imgproc.filter2D(frame, dest, -1, kernel, new Point(-1, -1), 127);
    }

    public void sharpen(Mat frame, Mat dest) {
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        kernel.put(0, 0, -.25f, -.5f, -.25f, -.5f, 4f, -.5f, -.25f, -.5f, -.25f);
        Imgproc.filter2D(frame, dest, -1, kernel);
    }

    public void vertical(Mat frame, Mat dest) {
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        kernel.put(0, 0, -.25f, -.5f, -.25f, -.5f, 4f, -.5f, -.25f, -.5f, -.25f);
        Imgproc.filter2D(frame, dest, -1, kernel);
    }

    public List<MatOfPoint> findObj(Mat frame) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    public List<Rect> drawBoxes(Mat frame, List<MatOfPoint> contours) {
        List<Rect> rects = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            Rect rectangle = Imgproc.boundingRect(contour);
            if (rectangle.width > 5 && rectangle.width < 150 && rectangle.height > 75) {
                rects.add(rectangle);
                Imgproc.rectangle(
                        frame,
                        new Point(rectangle.x, rectangle.y),
                        new Point(rectangle.x + rectangle.width,rectangle.y + rectangle.height),
                        new Scalar(36, 255, 12),
                        2
                );
            }
        }
        return rects;
    }

    public List<Double> findOffsets(Mat frame, List<Rect> rects) {
        List<Double> offsets = new ArrayList<>();
        for (Rect rect : rects) {
            double o = (.5 - (rect.x + rect.width/2.) / frame.width()) * C270Info.horizontalFOV;
            offsets.add(o);
            Imgproc.putText(
                    frame,
                    String.format(Locale.getDefault(), "%.2f°", o),
                    new Point(rect.x, rect.y + rect.height + 24),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1,
                    new Scalar(0, 255, 0),
                    4
            );
        }
        return offsets;
    }
}
