package org.firstinspires.ftc.teamcode.subsystem.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class AverageColorPipeline extends OpenCvPipeline {

    private volatile boolean render;
    private volatile Rect roi = null;
    private volatile Scalar average = null;

    public AverageColorPipeline(boolean render) {
        super();
        this.render = render;
    }
    public AverageColorPipeline(boolean render, Rect roi) {
        this(render);
        this.roi = roi;
    }

    public boolean isRendering() {
        return render;
    }
    public void setRender(boolean render) {
        this.render = render;
    }
    public Rect getRoi() {
        return roi;
    }
    public void setRoi(Rect roi) {
        this.roi = roi;
    }
    public Scalar getAverage() {
        return average;
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        if (roi == null) roi = new Rect(0, 0, mat.cols(), mat.rows());
    }

    @Override
    public Mat processFrame(Mat input) {
        average = roi == null ? Core.mean(input) : Core.mean(input.submat(roi));
        if (render && roi != null) Imgproc.rectangle(input, roi, average);
        return input;
    }
}
