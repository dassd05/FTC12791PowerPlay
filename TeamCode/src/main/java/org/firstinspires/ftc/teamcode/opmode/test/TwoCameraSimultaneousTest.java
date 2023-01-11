package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.vision.AverageColorPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.MultiplePipeline;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@TeleOp(group = "test")
public class TwoCameraSimultaneousTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        WebcamName cam1Name = hardwareMap.get(WebcamName.class, "Webcam 1");
        WebcamName cam2Name = hardwareMap.get(WebcamName.class, "Webcam 2");
        OpenCvWebcam camera1 = OpenCvCameraFactory.getInstance().createWebcam(cam1Name);
        OpenCvWebcam camera2 = OpenCvCameraFactory.getInstance().createWebcam(cam2Name);
        AverageColorPipeline averageColor1 = new AverageColorPipeline(true);
        AverageColorPipeline averageColor2 = new AverageColorPipeline(true);
        camera1.setPipeline(new MultiplePipeline(averageColor1, new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
//                telemetry.addLine(String.format("Camera <code>%s</code> \"%s\" (%s) Stats",
//                                cam1Name.getDeviceName(), hardwareMap.getNamesOf(cam1Name).iterator().next(), cam1Name))
                telemetry.addData(formatName(cam1Name) + " Average Color", averageColor1.getAverage().toString());
                telemetry.addData(formatName(cam1Name) + " Frame Rate (fps)", camera1.getFps());
                telemetry.addData(formatName(cam1Name) + " Theoretical Max FPS", camera1.getCurrentPipelineMaxFps());
                telemetry.addData(formatName(cam1Name) + " Pipeline Time (ms)", camera1.getPipelineTimeMs());
                telemetry.addData(formatName(cam1Name) + " Overhead Time (ms)", camera1.getOverheadTimeMs());
                telemetry.addData(formatName(cam1Name) + " Total Frame Time (ms)", camera1.getTotalFrameTimeMs());
                telemetry.update();
                return input;
            }
        }));
        camera2.setPipeline(new MultiplePipeline(averageColor2, new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
//                telemetry.addLine(String.format("Camera <code>%s</code> \"%s\" (%s) Stats",
//                                cam2Name.getDeviceName(), hardwareMap.getNamesOf(cam2Name).iterator().next(), cam2Name))
                telemetry.addData(formatName(cam2Name) + " Average Color", averageColor2.getAverage().toString());
                telemetry.addData(formatName(cam2Name) + " Frame Rate (fps)", camera2.getFps());
                telemetry.addData(formatName(cam2Name) + " Theoretical Max FPS", camera2.getCurrentPipelineMaxFps());
                telemetry.addData(formatName(cam2Name) + " Pipeline Time (ms)", camera2.getPipelineTimeMs());
                telemetry.addData(formatName(cam2Name) + " Overhead Time (ms)", camera2.getOverheadTimeMs());
                telemetry.addData(formatName(cam2Name) + " Total Frame Time (ms)", camera2.getTotalFrameTimeMs());
                telemetry.update();
                return input;
            }
        }));
        camera1.openCameraDevice();
        camera2.openCameraDevice();
        camera1.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
        camera2.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            sleep(10);
        }

        camera1.stopStreaming();
        camera2.stopStreaming();
        camera1.closeCameraDevice();
        camera2.closeCameraDevice();
    }

    public String formatName(WebcamName webcamName) {
        return String.format("Camera <code>%s</code> \"%s\" (%s)",
                webcamName.getDeviceName(), hardwareMap.getNamesOf(webcamName).iterator().next(), webcamName);
    }
}
