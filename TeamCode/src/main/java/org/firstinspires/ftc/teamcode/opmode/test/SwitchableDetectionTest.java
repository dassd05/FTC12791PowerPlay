//package org.firstinspires.ftc.teamcode.opmode.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.subsystem.Webcam;
//import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
//import org.firstinspires.ftc.teamcode.subsystem.vision.ConeDetectionPipeline;
//import org.firstinspires.ftc.teamcode.subsystem.vision.JunctionDetectionPipeline;
//import org.firstinspires.ftc.teamcode.subsystem.vision.Target;
//import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvSwitchableWebcam;
//
//@Config
//@TeleOp(group = "test")
//public class SwitchableDetectionTest extends LinearOpMode {
//    public static boolean detectingCones = true;
//    public static boolean blueCone = true;
//    public static boolean turretLocking = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = TelemetryUtil.initTelemetry(telemetry);
//        ConeDetectionPipeline conePipeline = new ConeDetectionPipeline(blueCone ? ConeDetectionPipeline.Color.BLUE : ConeDetectionPipeline.Color.RED);
//        JunctionDetectionPipeline junctionPipeline = new JunctionDetectionPipeline();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().
//                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 1");
//        WebcamName webcamName2 = hardwareMap.get(WebcamName.class, "Webcam 2");
//        OpenCvSwitchableWebcam cameras = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcamName1, webcamName2);
//        cameras.setPipeline(detectingCones ? conePipeline : junctionPipeline);
//        cameras.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                cameras.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboard.startCameraStream(cameras, 60);
//
//        Turret turret = null;
//        try {
//            turret = new Turret(hardwareMap);
//        } catch (IllegalArgumentException e) {
//            turretLocking = false;
//        }
//
//        // provide a new object rather than a null value so our stream refresh checker doesn't get tripped up if the first frame has no junctions
//        Target lastTarget = new Target(null, null, 0);
//
//        sleep(1000);
//        waitForStart();
//
//        while (opModeIsActive()) {
//            sleep(10);
//            boolean webcam1Selected = cameras.getActiveCamera() == webcamName1;
//            if (webcam1Selected != detectingCones) {
//                cameras.setActiveCamera(webcam1Selected ? webcamName2 : webcamName1);
//                cameras.setPipeline(detectingCones ? conePipeline : junctionPipeline);
//            }
//
//            if (detectingCones) {
//            if (pipeline.getColor() == ConeDetectionPipeline.Color.BLUE != blueCone) {
//                pipeline = new ConeDetectionPipeline(blueCone ? ConeDetectionPipeline.Color.BLUE : ConeDetectionPipeline.Color.RED);
//                webcam.cameras.setPipeline(pipeline);
//            }
//
//            int numTargets = pipeline.getCones().size();
//            Target target = pipeline.getClosestCone();
//
//            // has the stream updated yet, or is it still the same thing?
//            if (target == lastTarget) continue;
//
//            if (turretLocking && turret != null && numTargets > 0 && target != null) {
//                turret.setTurretTargetPosition(turret.quadratureEncoder.getCurrentPosition() + Turret.radiansToTicks(Math.toRadians(target.offset)));
//            }
//
//            telemetry.addData("Number of Cones", numTargets);
//            if (target != null) {
//                telemetry.addData("Cone Offset", target.offset);
//                telemetry.addData("Cone Size", target.rect.width + " x " + target.rect.height);
//            }
//            telemetry.addData("Turret Status", !turretLocking ? "disabled" : turret != null ? "enabled" : "Error: Not configured properly!");
//            if (turret != null && turretLocking) {
//                telemetry.addData("Turret Target (deg)", Math.toDegrees(turret.getTurretTargetPosition()));
//                telemetry.addData("Turret Position (deg)", Math.toDegrees(turret.quadratureEncoder.getCurrentPosition()));
//                turret.update();
//            }
//            telemetry.addData("Camera", webcamName.toString());
//            telemetry.addData("Frame Rate (fps)", cameras.getFps());
//            telemetry.addData("Theoretical Max FPS", cameras.getCurrentPipelineMaxFps());
//            telemetry.addData("Pipeline Time (ms)", cameras.getPipelineTimeMs());
//            telemetry.addData("Overhead Time (ms)", cameras.getOverheadTimeMs());
//            telemetry.addData("Total Frame Time (ms)", cameras.getTotalFrameTimeMs());
//            telemetry.update();
//
//            // if cone is null, then we basically won't try to update anything until a cone is detected.
//            // the stream refresh checker will see that there are no new frames until a cone is detected, which is okay with us
//            // tldr: we want the first frame to register if there's no cones, but subsequent frames without cones don't matter
//            lastCone = target;
//        }
//        while (opModeIsActive()) {
//            sleep(10);
//            int numJunctions = pipeline.getJunctions().size();
//            Target junction = pipeline.getClosestJunction();
//
//            // has the stream updated yet, or is it still the same thing?
//            if (junction == lastJunction) continue;
//
//            if (turretLocking && turret != null && numJunctions > 0 && junction != null) {
//                turret.setTurretTargetPosition(turret.quadratureEncoder.getCurrentPosition() + Turret.radiansToTicks(Math.toRadians(junction.offset)));
//            }
//
//            telemetry.addData("Number of Junctions", numJunctions);
//            if (junction != null) {
//                telemetry.addData("Junction Offset", junction.offset);
//                telemetry.addData("Junction Size", junction.rect.width + " x " + junction.rect.height);
//            }
//            telemetry.addData("Turret Status", !turretLocking ? "disabled" : turret != null ? "enabled" : "Error: Not configured properly!");
//            if (turret != null && turretLocking) {
//                telemetry.addData("Turret Target (deg)", Math.toDegrees(turret.getTurretTargetPosition()));
//                telemetry.addData("Turret Position (deg)", Math.toDegrees(turret.quadratureEncoder.getCurrentPosition()));
//                turret.update();
//            }
//            telemetry.addData("Camera", webcamName.toString());
//            telemetry.addData("Frame Rate (fps)", cameras.getFps());
//            telemetry.addData("Theoretical Max FPS", cameras.getCurrentPipelineMaxFps());
//            telemetry.addData("Pipeline Time (ms)", cameras.getPipelineTimeMs());
//            telemetry.addData("Overhead Time (ms)", cameras.getOverheadTimeMs());
//            telemetry.addData("Total Frame Time (ms)", cameras.getTotalFrameTimeMs());
//            telemetry.update();
//
//            // if junction is null, then we basically won't try to update anything until a junction is detected.
//            // the stream refresh checker will see that there are no new frames until a junction is detected, which is okay with us
//            // tldr: we want the first frame to register if there's no junctions, but subsequent frames without junctions don't matter
//            lastJunction = junction;
//        }
//    }
//}