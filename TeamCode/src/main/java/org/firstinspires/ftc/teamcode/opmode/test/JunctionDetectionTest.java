package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.subsystem.vision.JunctionDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.Target;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Config
@TeleOp(group = "test")
public class JunctionDetectionTest extends LinearOpMode {
    public static boolean turretLocking = false;
    public static int frameHeightPixels = 720;
    public static int frameWidthPixels = 960;
    public static boolean listAllJunctions = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);
        JunctionDetectionPipeline pipeline = new JunctionDetectionPipeline();
//        Webcam webcam = new Webcam(hardwareMap, "Webcam 2", pipeline);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
        OpenCvCamera cvCamera = OpenCvCameraFactory.getInstance().createWebcam(
                webcamName,
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        cvCamera.setPipeline(pipeline);
        cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cvCamera.startStreaming(frameWidthPixels, frameHeightPixels, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Warning: " + webcamName + " could not be opened. Error Code: " + errorCode);
            }
        });
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(cvCamera, 60);
        Turret turret = null;
        try {
            turret = new Turret(hardwareMap);
        } catch (IllegalArgumentException e) {
            turretLocking = false;
        }
        boolean lastTurretZeroed = false;
        ElapsedTime turretZeroedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // provide a new object rather than a null value so our stream refresh checker doesn't get tripped up if the first frame has no junctions
        Target lastJunction = new Target(null, null, 0);

        sleep(500);

        waitForStart();

        while (opModeIsActive()) {
            List<Target> junctions = pipeline.getJunctions();
            int numJunctions = junctions.size();
            Target junction = pipeline.getMostCenteredJunction();

            // has the stream updated yet, or is it still the same thing?
//            if (junction == lastJunction) continue;

            if (!lastTurretZeroed && turret != null && turret.zeroed()) {
                lastTurretZeroed = true;
                turretZeroedTimer.reset();
            }

            if (/*junction != lastJunction &&*/ turretLocking && turret != null && numJunctions > 0 && junction != null && turret.zeroed()
                    && Math.abs(turret.quadratureEncoder.getRawVelocity()) < 5 && turretZeroedTimer.time() > 2000) {
                turret.setTurretTargetPosition(-turret.quadratureEncoder.getCurrentPosition() - Turret.radiansToTicks(Math.toRadians(junction.offset)));
            }
            turretLocking = false;

            telemetry.addData("Number of Junctions", numJunctions);
            if (listAllJunctions)
                for (Target j : junctions)
                    telemetry.addData("Junction at " + j.offset, j.rect.toString());
            if (junction != null /*&& junction != lastJunction*/) {
                telemetry.addData("Target Junction Offset", junction.offset);
                telemetry.addData("Target Junction Size", junction.rect.width + " x " + junction.rect.height);
            }
//            telemetry.addData("Turret Status", !turretLocking ? "disabled" : turret != null ? "enabled" : "Error: Not configured properly!");
            telemetry.addData("Turret Status", turret != null ? "enabled" : "Error: Not configured properly!");
            if (turret != null) {
                telemetry.addData("Turret Zeroed", turret.zeroed());
//                telemetry.addData("Turret Target (deg)", Math.toDegrees(turret.getTurretTargetPosition()));
//                telemetry.addData("Turret Position (deg)", Math.toDegrees(turret.quadratureEncoder.getCurrentPosition()));
                telemetry.addData("Turret Target", turret.getTurretTargetPosition());
                telemetry.addData("Turret Position", -turret.quadratureEncoder.getCurrentPosition());
                telemetry.addData("Turret Velocity", -turret.quadratureEncoder.getRawVelocity());
                turret.update();
            }
            telemetry.addData("Camera", webcamName.toString());
            telemetry.addData("Frame Rate (fps)", cvCamera.getFps());
            telemetry.addData("Theoretical Max FPS", cvCamera.getCurrentPipelineMaxFps());
            telemetry.addData("Pipeline Time (ms)", cvCamera.getPipelineTimeMs());
            telemetry.addData("Overhead Time (ms)", cvCamera.getOverheadTimeMs());
            telemetry.addData("Total Frame Time (ms)", cvCamera.getTotalFrameTimeMs());
            telemetry.update();

            // if junction is null, then we basically won't try to update anything until a junction is detected.
            // the stream refresh checker will see that there are no new frames until a junction is detected, which is okay with us
            // tldr: we want the first frame to register if there's no junctions, but subsequent frames without junctions don't matter
            lastJunction = junction;
        }
    }
}