package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.vision.SignalDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@TeleOp(group = "test")
public class SignalDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);
        SignalDetectionPipeline pipeline = new SignalDetectionPipeline();
        Webcam webcam = new Webcam(hardwareMap, pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(webcam.cvCamera, 60);

        sleep(500);

        waitForStart();

        while (opModeIsActive()) {
            sleep(50);
            if (pipeline.average == null) continue;
            telemetry.addData("Color", pipeline.average.toString());
            telemetry.addData("Detection", pipeline.position);
            telemetry.addData("Camera", webcam.webcamName.toString());
            telemetry.addData("Frame Rate (fps)", webcam.cvCamera.getFps());
            telemetry.addData("Theoretical Max FPS", webcam.cvCamera.getCurrentPipelineMaxFps());
            telemetry.addData("Pipeline Time (ms)", webcam.cvCamera.getPipelineTimeMs());
            telemetry.addData("Overhead Time (ms)", webcam.cvCamera.getOverheadTimeMs());
            telemetry.addData("Total Frame Time (ms)", webcam.cvCamera.getTotalFrameTimeMs());
            telemetry.update();
        }
    }
}