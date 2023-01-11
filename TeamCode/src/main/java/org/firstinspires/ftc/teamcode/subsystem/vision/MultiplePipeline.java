package org.firstinspires.ftc.teamcode.subsystem.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class MultiplePipeline extends OpenCvPipeline {
    OpenCvPipeline[] pipelines;

    public MultiplePipeline(OpenCvPipeline... pipelines) {
        this.pipelines = pipelines;
    }

    @Override
    public Mat processFrame(Mat input) {
        for (OpenCvPipeline pipeline : pipelines) input = pipeline.processFrame(input);
        return input;
    }
}
