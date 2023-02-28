package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.openftc.apriltag.AprilTagDetection;

@Config
@TeleOp(group = "test")
public class AprilTagDetectionTest extends LinearOpMode {
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
    public static double tagsize = 0.166;

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        telemetry = TelemetryUtil.initTelemetry(telemetry);
        Webcam webcam = new Webcam(hardwareMap, pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(webcam.cvCamera, 60);

        sleep(500);

        waitForStart();

        while (opModeIsActive()) {
            sleep(50);
            telemetry.addData("Detection", "None");
            for(AprilTagDetection tag : pipeline.getLatestDetections()) {
                switch (tag.id) {
                    case 1:
                        telemetry.addData("Detection", "Left");
                        break;
                    case 2:
                        telemetry.addData("Detection", "Middle");
                        break;
                    case 3:
                        telemetry.addData("Detection", "Right");
                        break;
                }
            }
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
