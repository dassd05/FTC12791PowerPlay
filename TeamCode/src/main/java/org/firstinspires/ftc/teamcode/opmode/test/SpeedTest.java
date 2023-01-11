package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@TeleOp(group = "test")
public class SpeedTest extends LinearOpMode {
    public static boolean ENABLE_CAMERA = true;

    public Map<String, List> data = new HashMap<>();  // shared across threads, but they don't access shared data, so should be fine?

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        Robot robot = new Robot(hardwareMap, telemetry);

        int initialCapacity = 6000;  // 6000 capacity ~ 1 minute of 100 Hz
        data.put("Total Update Time", new ArrayList<Long>(initialCapacity));
        data.put("IMU Time", new ArrayList<Long>(initialCapacity));
        data.put("Bulk Cache Time", new ArrayList<Long>(initialCapacity));
        data.put("Butterfly Time", new ArrayList<Long>(initialCapacity));
        data.put("IntakeOuttake Time", new ArrayList<Long>(initialCapacity));
        data.put("Odometry Time", new ArrayList<Long>(initialCapacity));

        if (ENABLE_CAMERA) {
            // we're assuming camera is half as fast as main loop
            data.put("Frame Rate", new ArrayList<Float>(initialCapacity >> 1));
            data.put("Theoretical Max FPS", new ArrayList<Integer>(initialCapacity >> 1));
            data.put("Pipeline Time", new ArrayList<Integer>(initialCapacity >> 1));
            data.put("Overhead Time", new ArrayList<Integer>(initialCapacity >> 1));
            data.put("Total Frame Time", new ArrayList<Integer>(initialCapacity >> 1));

            robot.initWebcam(new OpenCvPipeline() {
                @Override
                public void init(Mat mat) {
                    super.init(mat);
                    robot.webcam.cvCamera.showFpsMeterOnViewport(true);
                }
                @Override
                public Mat processFrame(Mat input) {
//                    telemetry.addLine("Camera <code>" + robot.webcam.webcamName.toString() + "</code> Stats")
                    telemetry.addData("Frame Rate", log("Frame Rate", robot.webcam.cvCamera.getFps()) + " FPS");
                    telemetry.addData("Theoretical Max FPS", log("Theoretical Max FPS", robot.webcam.cvCamera.getCurrentPipelineMaxFps()) + " FPS");
                    telemetry.addData("Pipeline Time", log("Pipeline Time", robot.webcam.cvCamera.getPipelineTimeMs()) + " ms");
                    telemetry.addData("Overhead Time", log("Overhead Time", robot.webcam.cvCamera.getOverheadTimeMs()) + " ms");
                    telemetry.addData("Total Frame Time", log("Total Frame Time", robot.webcam.cvCamera.getTotalFrameTimeMs()) + " ms");
                    telemetry.update();
                    return input;
                }
            });
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Total Update Time", logTime("Total Update Time", () -> {
                robot.updatePosition();
                telemetry.addLine("General Robot Stats")
                        .addData("IMU Time", logTime("IMU Time", robot::updateIMU))
                        .addData("Bulk Cache Time", logTime("Bulk Cache Time", robot::clearCache))
                        .addData("Butterfly Time", logTime("Butterfly Time", robot.butterfly::update))
                        .addData("IntakeOuttake Time", logTime("IntakeOuttake Time", robot.intakeOuttake::update))
                        .addData("Odometry Time", logTime("Odometry Time", robot.odometry::update));
            }));
            telemetry.update();
        }
    }

    public long log(String key, long datum) {
        data.get(key).add(datum);
        return datum;
    }
    public int log(String key, int datum) {
        data.get(key).add(datum);
        return datum;
    }
    public float log(String key, float datum) {
        data.get(key).add(datum);
        return datum;
    }

    public String logTime(String key, Runnable action) {
        return log(key, Timer.time(action, TimeUnit.MICROSECONDS)) + " us";
    }
}
