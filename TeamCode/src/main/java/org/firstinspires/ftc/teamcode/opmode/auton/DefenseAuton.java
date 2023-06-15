package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.JUNCTIONS.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.opmode.teleop.TestTeleop;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.subsystem.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.JunctionDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.SignalDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.Target;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous (name = "Defense Auton", group = "0", preselectTeleOp = "WorldsTele")

public class DefenseAuton extends LinearOpMode {

    public static double P = .007, I = 7e-11 , D = 400;

    public double targetPos = 0.0;

    public double coneOffset = 47;
    public double webcamOffset = 50;
    public double safeClear = 170; //440 //TODO: fix
    //public double slidesUp = 1440;
    public double slidesUp = 663/*665*/;

    public int turret = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        final double FEET_PER_METER = 3.28084;

        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        double tagsize = 0.166;

        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 60);

        boolean firstTime = true;

        Pose2d pose1 = new Pose2d(0,43,0);
        Pose2d pose2 = new Pose2d(-9,56,70);
        Pose2d pose3 = new Pose2d(-8.5, 61.5, 117);

        int cycles = -1;

        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime visionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeInInit()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        if (tag.id == LEFT)
                            telemetry.addLine("detected position: left");
                        if (tag.id == MIDDLE)
                            telemetry.addLine("detected position: middle");
                        if (tag.id == RIGHT)
                            telemetry.addLine("detected position: right");
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else {
                        telemetry.addLine("no april tag detected...");
                    }
                }
            }
            telemetry.update();

            robot.intakeOuttake.arm.claw.setPosition(.31);
            robot.intakeOuttake.arm.arm.setPosition(.53);
            robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
            robot.intakeOuttake.arm.setAligner(false);

            robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
            robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
            robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
            robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

            robot.update();
        }

        double totalError = 0.0;
        double lastError = 0.0;

        waitForStart();
        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
        myTimer.reset();

        boolean reached = false;

        long lastTime = System.nanoTime();

        Target junction = null, lastJunction = null;

        waitForStart();
        resetRuntime();

        state = State.FORWARD;
        score = Score.UP;

        robot.butterfly.setState(Butterfly.State.MECANUM);
        camera.stopStreaming();

        while (opModeIsActive()) {

            Pose2d poseEstimate = drive.getPoseEstimate();

            switch (state) {
                case FORWARD:
                    if (!reached) {
                        robot.butterfly.runToPosition(pose1.getX(), pose1.getY(), pose1.getHeading(),
                                .95, .85, -poseEstimate.getY(), poseEstimate.getX(),
                                poseEstimate.getHeading());
                        if (robot.butterfly.positionReached || myTimer.time() > 1750)
                            reached = true;
                    }
                    if (reached) {
                        robot.butterfly.runToPosition(pose2.getX(), pose2.getY(), pose2.getHeading(),
                                .35, .9, -poseEstimate.getY(), poseEstimate.getX(),
                                poseEstimate.getHeading(), true);

                        robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);

                        if (robot.butterfly.positionReached || myTimer.time() > 2500) {
                            reached = false;
                            myTimer.reset();
                            state = State.POSITION;
                        }
                    }

                    if (myTimer.time() > 700)
                        robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);

                    break;

                case POSITION:
                    robot.butterfly.runToPosition(pose3.getX(), pose3.getY(), pose3.getHeading(),
                            .6, .95, -poseEstimate.getY(), poseEstimate.getX(),
                            poseEstimate.getHeading());
                    if (robot.butterfly.positionReached || myTimer.time() > 2000)
                        myTimer.reset();
                        state = State.SCORE;
                    break;
                case SCORE:
                    robot.butterfly.runToPosition(pose3.getX(), pose3.getY(), pose3.getHeading(),
                            .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
                            poseEstimate.getHeading(), true);
                    break;
            }


            long time = System.nanoTime();
            double error = targetPos - robot.vertical.v2.getCurrentPosition();
            if (error < 0 != lastError < 0) totalError = 0;
            else totalError += (error) * (time - lastTime);
            double d = (error - lastError) / (time - lastTime);

            double pError = P * error;
            double i = totalError * I;
            double iError = (Math.abs(i) < .4 ? i : Math.signum(i) * .4);
            double dError = D * d;

            double power = (pError + iError + dError);
            lastTime = time;
            lastError = error;

            robot.vertical.v1.setPower(power + .01);
            robot.vertical.v2.setPower(power + .01);
            robot.vertical.v3.setPower(power + .01);


            PoseStorage.currentPose = poseEstimate;

            if (getRuntime() > 29.95) {
                robot.turret.motor.setPower(0);
                robot.butterfly.drive(0,0,0);
                break;
            }

            if (state != State.PARK)
                robot.turret.setTurretTargetPosition(turret);
            else
                robot.turret.motor.setPower(0);

            telemetry.addData("pose heading", poseEstimate.getHeading());

            robot.update();
            drive.update();
            telemetry.update();
        }
    }
    public enum State {
        FORWARD,
        POSITION,
        SCORE,
        PARK
    }
    public State state;

    public enum Score {
        EXTEND,
        GRAB,
        LIFT,
        DEEXTEND,
        UP,
        EXTENDSCORE,
        REST,
        FINISH
    }
    public Score score;

    public double linearProfile(double totalTime, double currentTime, double endTime, double initial, double fin) {
        return (initial + (fin - initial) * (1 - (endTime - currentTime) / totalTime));
    }
}
