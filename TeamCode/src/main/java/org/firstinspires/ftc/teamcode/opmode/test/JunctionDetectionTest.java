package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.subsystem.vision.JunctionDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.Target;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Config
@TeleOp(group = "test")
public class JunctionDetectionTest extends LinearOpMode {
    public static boolean turretLocking = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);
        JunctionDetectionPipeline pipeline = new JunctionDetectionPipeline();
        Webcam webcam = new Webcam(hardwareMap, "Webcam 2", pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(webcam.cvCamera, 60);
        Turret turret = null;
        try {
            turret = new Turret(hardwareMap);
        } catch (IllegalArgumentException e) {
            turretLocking = false;
        }

        // provide a new object rather than a null value so our stream refresh checker doesn't get tripped up if the first frame has no junctions
        Target lastJunction = new Target(null, null, 0);

        sleep(500);

        waitForStart();

        while (opModeIsActive()) {
            sleep(10);
            int numJunctions = pipeline.getJunctions().size();
            Target junction = pipeline.getClosestJunction();

            // has the stream updated yet, or is it still the same thing?
            if (junction == lastJunction) continue;

            if (turretLocking && turret != null && numJunctions > 0 && junction != null && turret.zeroed()) {
                turret.setTurretTargetPosition(-turret.quadratureEncoder.getCurrentPosition() + Turret.radiansToTicks(Math.toRadians(junction.offset)));
            }

            telemetry.addData("Number of Junctions", numJunctions);
            if (junction != null) {
                telemetry.addData("Junction Offset", junction.offset);
                telemetry.addData("Junction Size", junction.rect.width + " x " + junction.rect.height);
            }
            telemetry.addData("Turret Status", !turretLocking ? "disabled" : turret != null ? "enabled" : "Error: Not configured properly!");
            if (turret != null && turretLocking) {
                telemetry.addData("Turret Zeroed", turret.zeroed());
//                telemetry.addData("Turret Target (deg)", Math.toDegrees(turret.getTurretTargetPosition()));
//                telemetry.addData("Turret Position (deg)", Math.toDegrees(turret.quadratureEncoder.getCurrentPosition()));
                telemetry.addData("Turret Target", turret.getTurretTargetPosition());
                telemetry.addData("Turret Position", -turret.quadratureEncoder.getCurrentPosition());
                turret.update();
            }
            telemetry.addData("Camera", webcam.webcamName.toString());
            telemetry.addData("Frame Rate (fps)", webcam.cvCamera.getFps());
            telemetry.addData("Theoretical Max FPS", webcam.cvCamera.getCurrentPipelineMaxFps());
            telemetry.addData("Pipeline Time (ms)", webcam.cvCamera.getPipelineTimeMs());
            telemetry.addData("Overhead Time (ms)", webcam.cvCamera.getOverheadTimeMs());
            telemetry.addData("Total Frame Time (ms)", webcam.cvCamera.getTotalFrameTimeMs());
            telemetry.update();

            // if junction is null, then we basically won't try to update anything until a junction is detected.
            // the stream refresh checker will see that there are no new frames until a junction is detected, which is okay with us
            // tldr: we want the first frame to register if there's no junctions, but subsequent frames without junctions don't matter
            lastJunction = junction;
        }
    }
}