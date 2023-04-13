package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous (name = "MTI Auton", group = "0", preselectTeleOp = "altMTITele")

public class MTIAuton extends LinearOpMode {

    public static double P = .0035, I = .000000000001 , D = 25;

    public double targetPos = 0.0;

    public double coneOffset = 84.5;
    public double webcamOffset = 175;
    public double safeClear = 440; //440
    //public double slidesUp = 1440;
    public double slidesUp = 1125;


    public int turret = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        SignalDetectionPipeline pipeline = new SignalDetectionPipeline();
//        Webcam webcam = new Webcam(hardwareMap, pipeline);
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        //JunctionDetectionPipeline junctionDetectionPipeline;

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
        //WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        //WebcamName webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //junctionDetectionPipeline = new JunctionDetectionPipeline();
        //camera.openCameraDevice();
        //camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
        //camera.setActiveCamera(webcam1);
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

        //SignalDetectionPipeline.ParkPosition parkPosition = SignalDetectionPipeline.ParkPosition.MIDDLE;

        double rightOffset = 75.5;

        drive.setPoseEstimate(new Pose2d(0, -rightOffset,0));

        Pose2d pose1 = new Pose2d(0 + rightOffset,60,0);
        Pose2d pose1_2 = new Pose2d(-3 + rightOffset,47,-45);
        Pose2d pose2 = new Pose2d(5 + rightOffset,50.5,-90);

        Pose2d left = new Pose2d(-24 + rightOffset,50.5,0);
        Pose2d right = new Pose2d(24 + rightOffset,50.5,0);
        Pose2d middle = new Pose2d(0 + rightOffset,50.5,0);

        int cycles = -1;

        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
            //sleep(250);
//            parkPosition = pipeline.position;
//            telemetry.addData("position", parkPosition);
//            if (pipeline.average != null) telemetry.addData("color", pipeline.average.toString());
            telemetry.update();

            robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
            robot.intakeOuttake.arm.arm.setPosition(.53);
            robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);

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
//        camera.setActiveCamera(webcam2);
//        camera.setPipeline(junctionDetectionPipeline);

        while (opModeIsActive()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
//            lastJunction = junction;
//            junction = junctionDetectionPipeline.getClosestJunction();

            switch (state) {
                case FORWARD:
                    if (!reached) {
                        robot.butterfly.runToPosition(pose1.getX(), pose1.getY(), pose1.getHeading(),
                                .9, .9, -poseEstimate.getY(), poseEstimate.getX(),
                                poseEstimate.getHeading());
                        if (robot.butterfly.positionReached || myTimer.time() > 2000)
                            reached = true;
                    }
                    if (reached) {
                        robot.butterfly.runToPosition(pose1_2.getX(), pose1_2.getY(), pose1_2.getHeading(),
                                .9, .4, -poseEstimate.getY(), poseEstimate.getX(),
                                poseEstimate.getHeading());

                        robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);

                        //robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
                        //robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);

                        if (robot.butterfly.positionReached || myTimer.time() > 3000) {
                            myTimer.reset();
                            state = State.POSITION;
                        }
                    }
                    break;

                case POSITION:
//                    robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
//                    robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);

                    robot.butterfly.runToPosition(pose2.getX(), pose2.getY(), pose2.getHeading(),
                            .45, 1, -poseEstimate.getY(), poseEstimate.getX(),
                            poseEstimate.getHeading(), true);
                    if (robot.butterfly.positionReached || myTimer.time() > 2000) {
                        myTimer.reset();
                        state = State.SCORE;
                    }
                    break;

                case SCORE:
                    //robot.butterfly.setState(Butterfly.State.STANDSTILL);
                    robot.butterfly.runToPosition(pose2.getX(), pose2.getY(), pose2.getHeading(),
                            .10, .3, -poseEstimate.getY(), poseEstimate.getX(),
                            poseEstimate.getHeading(), true);

                    switch (score) {
                        case EXTEND:
                            targetPos = (5 - cycles) * coneOffset + webcamOffset;

                            if (myTimer.time() < 700) {
                                if (cycles == -1) {
                                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(700, myTimer.time(), 700, BACKWARD_LEFT_IN, (BACKWARD_LEFT_OUT - .074)));
                                    robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(700, myTimer.time(), 700, BACKWARD_RIGHT_IN, BACKWARD_RIGHT_OUT + .074));
                                } else if (cycles < 6) {
                                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(700, myTimer.time(), 700, ((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2), (BACKWARD_LEFT_OUT - .074)));
                                    robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(700, myTimer.time(), 700, ((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2), BACKWARD_RIGHT_OUT + .074));
                                }
                            } else {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_OUT - .074);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_OUT + .074);
                            }

                            //if (myTimer.time() > 200) {
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition((((FORWARD_RIGHT_IN + FORWARD_RIGHT_OUT) / 2) + (FORWARD_RIGHT_IN)) / 2);
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition((((FORWARD_LEFT_IN + FORWARD_LEFT_OUT) / 2) + (FORWARD_LEFT_IN)) / 2);

//                            if (myTimer.time() > 300) {
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
//                            } else {
                            if (myTimer.time() > 325) {
                                robot.intakeOuttake.horizontal.forwardRight.setPosition((((FORWARD_RIGHT_IN + FORWARD_RIGHT_OUT) / 2) + (FORWARD_RIGHT_IN)) / 2);
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition((((FORWARD_LEFT_IN + FORWARD_LEFT_OUT) / 2) + (FORWARD_LEFT_IN)) / 2);
                            }
//                            }
//                            }
//                            else {
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition((FORWARD_RIGHT_IN + FORWARD_RIGHT_OUT) / 2);
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition((FORWARD_LEFT_IN + FORWARD_LEFT_OUT) / 2);
//                            }


                            if (cycles < 0) {
                                if (myTimer.time() > 50 && myTimer.time() < 450)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, myTimer.time(), 450, (ARM_REST + ARM_INTAKE) / 2, ARM_INTAKE));
                                else if (myTimer.time() >= 450)
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE);
                            } else if (cycles < 1) {
                                if (myTimer.time() > 50 && myTimer.time() < 450)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, myTimer.time(), 450, (ARM_REST + ARM_INTAKE) / 2, ARM_INTAKE + .006));
                                else if (myTimer.time() >= 450)
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE + .008);
                            } else {
                                if (myTimer.time() > 50 && myTimer.time() < 450)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, myTimer.time(), 450, (ARM_REST + ARM_INTAKE) / 2, ARM_INTAKE + .01));
                                else if (myTimer.time() >= 450)
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE + .01);
                            }

                            robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);

                            if (myTimer.time() > 50)
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);

                            if (myTimer.time() > 785) {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_OUT - .074);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_OUT + .074);
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                                myTimer.reset();
                                reached = true;
                                score = Score.GRAB;
                            }
                            break;
                        case GRAB:

                            if (myTimer.time() < 625) {
                                robot.intakeOuttake.horizontal.forwardRight.setPosition((((FORWARD_RIGHT_IN + FORWARD_RIGHT_OUT) / 2) + (FORWARD_RIGHT_IN)) / 2);
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition((((FORWARD_LEFT_IN + FORWARD_LEFT_OUT) / 2) + (FORWARD_LEFT_IN)) / 2);

                            } else {
                                robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                            }

                            robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                            if (myTimer.time() > 375 && reached) {
                                targetPos += safeClear;
                                reached = false;
                            }
                            if (myTimer.time() > 460) {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
//                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(400, myTimer.time(), 850,BACKWARD_LEFT_OUT - .144, BACKWARD_LEFT_IN));
//                                robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(400, myTimer.time(), 850, BACKWARD_RIGHT_OUT + .144, BACKWARD_RIGHT_IN));
                            }

                            if( myTimer.time() >= 550 && myTimer.time() <= 1025) {
                                robot.intakeOuttake.arm.arm.setPosition((ARM_INTAKE + ((ARM_REST + ARM_OUTTAKE) / 2 - ARM_INTAKE) * (1 - (950 - myTimer.time()) / 475)));
                            }

                            if (myTimer.time() > 1025) {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

                                turret = 420;
                                robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);
                                myTimer.reset();
                                score = Score.UP;
                            }

                            break;
                        case UP:
                            turret = 420;
                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

                            robot.intakeOuttake.horizontal.forwardRight.setPosition((((FORWARD_RIGHT_IN + FORWARD_RIGHT_OUT) / 2) + (FORWARD_RIGHT_IN)) / 2);
                            robot.intakeOuttake.horizontal.forwardLeft.setPosition((((FORWARD_LEFT_IN + FORWARD_LEFT_OUT) / 2) + (FORWARD_LEFT_IN)) / 2);
//                            robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
//                            robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                            targetPos = slidesUp;

                            robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                            if (myTimer.time() > 600) {
                                myTimer.reset();
                                score = Score.EXTENDSCORE;
                            }
                            break;
                        case EXTENDSCORE:
                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

//                            if (myTimer.time() < 500) {
//                                if (junction != lastJunction && junction != null && Math.abs(robot.turret.quadratureEncoder.getRawVelocity()) < 5) {
//                                    turret = -robot.turret.quadratureEncoder.getCurrentPosition() + Turret.radiansToTicks(Math.toRadians(junction.offset));
//                                }
//                            }

                            if (myTimer.time() <= 475) {
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(((FORWARD_LEFT_OUT + FORWARD_LEFT_IN) / 2 - .03));
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition(((FORWARD_RIGHT_OUT + FORWARD_RIGHT_IN) / 2 + .03));
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(((FORWARD_LEFT_OUT + FORWARD_LEFT_IN) / 2 - .065));
                                robot.intakeOuttake.horizontal.forwardRight.setPosition(((FORWARD_RIGHT_OUT + FORWARD_RIGHT_IN) / 2 + .065));

//                                //robot.intakeOuttake.horizontal.forwardLeft.setPosition(linearProfile(650, myTimer.time(), 650,((((FORWARD_LEFT_IN + FORWARD_LEFT_OUT) / 2) + (FORWARD_LEFT_IN)) / 2), ((FORWARD_LEFT_OUT + FORWARD_LEFT_IN) / 2 - .025)));
                                //robot.intakeOuttake.horizontal.forwardRight.setPosition(linearProfile(650, myTimer.time(), 650, ((((FORWARD_RIGHT_IN + FORWARD_RIGHT_OUT) / 2) + (FORWARD_RIGHT_IN)) / 2), ((FORWARD_RIGHT_OUT + FORWARD_RIGHT_IN) / 2 + .025)));
                            }

                            if (myTimer.time() <= 475)
                                //robot.intakeOuttake.arm.arm.setPosition(((ARM_REST + ARM_OUTTAKE) / 2) + (ARM_OUTTAKE - ((ARM_REST + ARM_OUTTAKE) / 2)) * (1 - (650 - myTimer.time()) / 650));
                                robot.intakeOuttake.arm.arm.setPosition(((ARM_REST + ARM_OUTTAKE) / 2) + (.31 - ((ARM_REST + ARM_OUTTAKE) / 2)) * (1 - (475 - myTimer.time()) / 475));
                            else if (myTimer.time() < 525)
                                //robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE);
                                robot.intakeOuttake.arm.arm.setPosition(.31);


                            if (myTimer.time() > 525) {
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition((FORWARD_LEFT_OUT + FORWARD_LEFT_IN) / 2 - .03);
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition((FORWARD_RIGHT_OUT + FORWARD_RIGHT_IN) / 2 + .03);
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(((FORWARD_LEFT_OUT + FORWARD_LEFT_IN) / 2 - .065));
                                robot.intakeOuttake.horizontal.forwardRight.setPosition(((FORWARD_RIGHT_OUT + FORWARD_RIGHT_IN) / 2 + .065));

                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE - .1);

                                if (myTimer.time() > 700) {
                                    targetPos -= 500;
                                    myTimer.reset();
                                    score = Score.REST;
                                }
                            }
                            break;
                        case REST:
                            if (myTimer.time() < 400)
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE);

//                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
//                            robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                            robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
                            robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);

                            if (myTimer.time() > 100 && myTimer.time() < 400)
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                            if (myTimer.time() > 300) {

                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(linearProfile(600, myTimer.time(), 900, ((FORWARD_LEFT_OUT + FORWARD_LEFT_IN) / 2 - .065), ((((FORWARD_LEFT_IN + FORWARD_LEFT_OUT) / 2) + (FORWARD_LEFT_IN)) / 2)));
                                robot.intakeOuttake.horizontal.forwardRight.setPosition(linearProfile(600, myTimer.time(), 900, ((FORWARD_RIGHT_OUT + FORWARD_RIGHT_IN) / 2 + .065), ((((FORWARD_RIGHT_IN + FORWARD_RIGHT_OUT) / 2) + (FORWARD_RIGHT_IN)) / 2)));

                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_SAFE);
                            }

                            if (myTimer.time() > 450)
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                            if (myTimer.time() > 650) {
                                turret = 0;
                                targetPos = (4 - cycles) * coneOffset + webcamOffset;
                            }

                            if (myTimer.time() >= 400 && myTimer.time() <= 1000) {
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + (ARM_REST - ARM_OUTTAKE) * (1 - (1000 - myTimer.time()) / 600));
                            }

                            if (myTimer.time() > 1000) {
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition(((FORWARD_RIGHT_IN + FORWARD_RIGHT_OUT) / 2 + FORWARD_RIGHT_IN) / 2);
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(((FORWARD_LEFT_IN + FORWARD_LEFT_OUT) / 2 + FORWARD_LEFT_IN) / 2);

                                robot.intakeOuttake.horizontal.forwardRight.setPosition((FORWARD_RIGHT_IN));
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition((FORWARD_LEFT_IN));

                                robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_INTAKE) / 2);
                                cycles += 1;
                                myTimer.reset();
                                score = Score.EXTEND;
                            }
                            break;
                    }
                    if (cycles == 5)
                        state = State.PARK;
                    break;

                case PARK:
                    robot.butterfly.setState(Butterfly.State.MECANUM);

                    robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                    robot.intakeOuttake.arm.arm.setPosition(ARM_REST);
                    robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                    targetPos = 0;
                    turret = 0;

                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                    switch (tagOfInterest.id) {
                        case 1:
                            robot.butterfly.runToPosition(left.getX(), left.getY(), left.getHeading(),
                                    .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
                                    poseEstimate.getHeading());
                            break;

                        case 3:
                            robot.butterfly.runToPosition(right.getX(), right.getY(), right.getHeading(),
                                    .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
                                    poseEstimate.getHeading());
                            break;

                        case 2:
                            robot.butterfly.runToPosition(middle.getX(), middle.getY(), middle.getHeading(),
                                    .85, 1, -poseEstimate.getY(), poseEstimate.getX(),
                                    poseEstimate.getHeading());
                            break;
                    }
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
            robot.vertical.v3.setPower(power +.01);


            PoseStorage.currentPose = poseEstimate;

            if (getRuntime() > 29.95) {
                robot.turret.motor.setPower(0);
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

