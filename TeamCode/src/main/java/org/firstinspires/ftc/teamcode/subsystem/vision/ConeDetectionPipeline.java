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
public class ConeDetectionPipeline extends OpenCvPipeline {
    public static boolean maskedView = false;
    public static int blueLowH = 0;//108;
    public static int blueLowS = 100;//150;
    public static int blueLowV = 50;
    public static int blueHighH = 10;//115;
    public static int blueHighS = 255;
    public static int blueHighV = 255;//200;
    public static int redLowH = 115;//0;
    public static int redLowS = 100;//130;
    public static int redLowV = 85;//100;
    public static int redHighH = 120;//5;
    public static int redHighS = 255;
    public static int redHighV = 255;

    private final Color color;
    private final Object coneLock = new Object();

    private List<Target> cones;

    public enum Color {
        BLUE,
        RED,
    }

    public static class Target {
        public final MatOfPoint contour;
        public final Rect rect;
        public final double offset;

        public Target(MatOfPoint contour, Rect rect, double offset) {
            this.contour = contour;
            this.rect = rect;
            this.offset = offset;
        }
    }

    public ConeDetectionPipeline(Color color) {
        this.color = color;
    }

    public Color getColor() {
        return color;
    }
    public List<Target> getCones() {
        synchronized (coneLock) {
            return new ArrayList<>(cones);
        }
    }
    @Nullable public Target getClosestCone() {
        synchronized (coneLock) {
            if (cones.size() == 0) return null;
            Target largest = cones.get(0);
            for (Target cone : cones) {
                if (cone.rect.area() > largest.rect.area()) largest = cone;
            }
            return largest;
        }
    }
    @Nullable public Target getMostCenteredCone() {
        synchronized (coneLock) {
            if (cones.size() == 0) return null;
            Target closest = cones.get(0);
            for (Target cone : cones) {
                if (Math.abs(cone.offset) < Math.abs(closest.offset)) closest = cone;
            }
            return closest;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat copy = new Mat(input.size(), CvType.CV_8UC1);
        switch (color) {
            case BLUE: inRangeBlue(input, copy); break;
            case RED: inRangeRed(input, copy); break;
        }
        List<MatOfPoint> contours = findObj(copy);
        List<Rect> rects = drawBoxes(input, contours);
        List<Double> offsets = findOffsets(input, rects);

        synchronized (coneLock) {
            cones = new ArrayList<>();
            for (int i = 0; i < rects.size(); i++)
                cones.add(new Target(contours.get(i), rects.get(i), offsets.get(i)));
        }

        return maskedView ? copy : input;
    }

    public void inRangeBlue(Mat frame, Mat dest) {
        Imgproc.cvtColor(frame, dest, Imgproc.COLOR_BGR2HSV);
        Core.inRange(dest, new Scalar(blueLowH, blueLowS, blueLowV), new Scalar(blueHighH, blueHighS, blueHighV), dest);
    }

    public void inRangeRed(Mat frame, Mat dest) {
        Imgproc.cvtColor(frame, dest, Imgproc.COLOR_BGR2HSV);
        Core.inRange(dest, new Scalar(redLowH, redLowS, redLowV), new Scalar(redHighH, redHighS, redHighV), dest);
    }

    public void sharpen(Mat frame, Mat dest) {
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
            if (rectangle.width > 64 && rectangle.height > 80) {
                rects.add(rectangle);
                Imgproc.rectangle(
                        frame,
                        new Point(rectangle.x, rectangle.y),
                        new Point(rectangle.x + rectangle.width,rectangle.y + rectangle.height),
                        new Scalar(36, 255, 12),
                        3
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
                    new Point(rect.x, rect.y - 6),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    16,
                    new Scalar(0, 255, 0),
                    65
            );
        }
        return offsets;
    }
}
