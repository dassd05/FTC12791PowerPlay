package org.firstinspires.ftc.teamcode.subsystem.vision;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;

public class Target {
    public final MatOfPoint contour;
    public final Rect rect;
    public final double offset;

    public Target(MatOfPoint contour, Rect rect, double offset) {
        this.contour = contour;
        this.rect = rect;
        this.offset = offset;
    }
}