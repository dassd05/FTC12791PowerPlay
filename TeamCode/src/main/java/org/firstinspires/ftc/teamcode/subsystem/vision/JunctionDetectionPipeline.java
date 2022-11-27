package org.firstinspires.ftc.teamcode.subsystem.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class JunctionDetectionPipeline extends OpenCvPipeline {

    public volatile double angleOff;

    @Override
    public Mat processFrame(Mat input) {

        return input;
    }
}
