package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.subsystem.vision.ConeDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.Target;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Config
@TeleOp(group = "test")
public class ConeDetectionTest extends LinearOpMode {
    public static boolean blueCone = true;
    public static boolean turretLocking = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);
        ConeDetectionPipeline pipeline = new ConeDetectionPipeline(blueCone ? ConeDetectionPipeline.Color.BLUE : ConeDetectionPipeline.Color.RED);
        Webcam webcam = new Webcam(hardwareMap, pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(webcam.cvCamera, 60);
        Turret turret = null;
        try {
            turret = new Turret(hardwareMap);
        } catch (IllegalArgumentException e) {
            turretLocking = false;
        }

        // provide a new object rather than a null value so our stream refresh checker doesn't get tripped up if the first frame has no cones
        Target lastCone = new Target(null, null, 0);

        sleep(500);

        waitForStart();

        while (opModeIsActive()) {
            sleep(10);
            if (pipeline.getColor() == ConeDetectionPipeline.Color.BLUE != blueCone) {
                pipeline = new ConeDetectionPipeline(blueCone ? ConeDetectionPipeline.Color.BLUE : ConeDetectionPipeline.Color.RED);
                webcam.cvCamera.setPipeline(pipeline);
            }

            int numCones = pipeline.getCones().size();
            Target cone = pipeline.getClosestCone();

            // has the stream updated yet, or is it still the same thing?
            if (cone == lastCone) continue;

            if (turretLocking && turret != null && numCones > 0 && cone != null && turret.zeroed()) {
                turret.setTurretTargetPosition(-turret.quadratureEncoder.getCurrentPosition() + Turret.radiansToTicks(Math.toRadians(cone.offset)));
            }

            telemetry.addData("Number of Cones", numCones);
            if (cone != null) {
                telemetry.addData("Cone Offset", cone.offset);
                telemetry.addData("Cone Size", cone.rect.width + " x " + cone.rect.height);
            }
            telemetry.addData("Turret Status", !turretLocking ? "disabled" : turret != null ? "enabled" : "Error: Not configured properly!");
            if (turret != null && turretLocking) {
                telemetry.addData("Turret Target (deg)", Math.toDegrees(turret.getTurretTargetPosition()));
                telemetry.addData("Turret Position (deg)", Math.toDegrees(turret.quadratureEncoder.getCurrentPosition()));
                turret.update();
            }
            telemetry.addData("Camera", webcam.webcamName.toString());
            telemetry.addData("Frame Rate (fps)", webcam.cvCamera.getFps());
            telemetry.addData("Theoretical Max FPS", webcam.cvCamera.getCurrentPipelineMaxFps());
            telemetry.addData("Pipeline Time (ms)", webcam.cvCamera.getPipelineTimeMs());
            telemetry.addData("Overhead Time (ms)", webcam.cvCamera.getOverheadTimeMs());
            telemetry.addData("Total Frame Time (ms)", webcam.cvCamera.getTotalFrameTimeMs());
            telemetry.update();

            // if cone is null, then we basically won't try to update anything until a cone is detected.
            // the stream refresh checker will see that there are no new frames until a cone is detected, which is okay with us
            // tldr: we want the first frame to register if there's no cones, but subsequent frames without cones don't matter
            lastCone = cone;
        }
    }
}