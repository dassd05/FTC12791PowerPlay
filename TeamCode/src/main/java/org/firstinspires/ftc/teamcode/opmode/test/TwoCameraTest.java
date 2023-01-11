package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

@Config
@TeleOp(group = "test")
public class TwoCameraTest extends LinearOpMode {
    public static boolean chooseCamera = true;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName cam1Name = hardwareMap.get(WebcamName.class, "Webcam 1");
        WebcamName cam2Name = hardwareMap.get(WebcamName.class, "Webcam 2");
        OpenCvSwitchableWebcam camera = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, cam1Name, cam2Name);
        camera.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
//                telemetry.addLine("Camera <code>" + robot.webcam.webcamName.toString() + "</code> Stats")
                telemetry.addData("Current Active Camera", camera.getActiveCamera().toString());
                telemetry.addData("Frame Rate (fps)", camera.getFps());
                telemetry.addData("Theoretical Max FPS", camera.getCurrentPipelineMaxFps());
                telemetry.addData("Pipeline Time (ms)", camera.getPipelineTimeMs());
                telemetry.addData("Overhead Time (ms)", camera.getOverheadTimeMs());
                telemetry.addData("Total Frame Time (ms)", camera.getTotalFrameTimeMs());
                telemetry.update();
                return input;
            }
        });
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 60);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        JustPressed justPressed = new JustPressed(gamepad1);
        waitForStart();

        while (opModeIsActive()) {
            if (justPressed.a()) chooseCamera = !chooseCamera;
            if (chooseCamera) camera.setActiveCamera(cam1Name);
            else camera.setActiveCamera(cam2Name);

            justPressed.update();
            sleep(10);
        }

        FtcDashboard.getInstance().stopCameraStream();
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
