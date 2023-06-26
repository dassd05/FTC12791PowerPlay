package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.JUNCTIONS.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.subsystem.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Regular Left", group = "0", preselectTeleOp = "FinalMTITele")

public class NewMTILeftAuto extends LinearOpMode {

    public static Vector2d stack = new Vector2d(-32, 49.25);

    public static double P = .007, I = 7e-11 , D = 400;

    public double targetPos = 50.0;

    public boolean back = true;

    public double distance = 0;

    public double coneOffset = 47;
    public double webcamOffset = 50 + 20;
    public double safeClear = 170; //440 //TODO: fix
    //public double slidesUp = 1440;
    public double slidesUp = 655/*665*/;

    public int turret = 0;

    public Vector2d junction = B3;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(new Pose2d(-1,0,0));

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

        Pose2d pose1 = new Pose2d(0,60,0);
        Pose2d pose1_2 = new Pose2d(3,46,45);
        Pose2d pose2 = new Pose2d(-8,49.5,90);

        Pose2d left = new Pose2d(-24,49.5,0);
        Pose2d right = new Pose2d(24,49.5,0);
        Pose2d middle = new Pose2d(0,49.5,0);

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
        //robot.intakeOuttake.arm.aligner.setPwmDisable();

        myTimer.reset();

        boolean reached = false;

        long lastTime = System.nanoTime();

        resetRuntime();

        state = State.FORWARD;
        score = Score.UP;

        robot.butterfly.setState(Butterfly.State.MECANUM);
        camera.stopStreaming();

        while (opModeIsActive()) {

            Pose2d poseEstimate = drive.getPoseEstimate();

            switch (state) {
                case FORWARD:
//                    if (myTimer.time() > 300)
//                        robot.intakeOuttake.arm.claw.setPwmDisable();
                    if (!reached) {
                        robot.butterfly.runToPosition(pose1.getX(), pose1.getY(), pose1.getHeading(),
                                .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
                                poseEstimate.getHeading());
                        if (robot.butterfly.positionReached || myTimer.time() > 2000)
                            reached = true;
                    }
                    if (reached) {
                        robot.butterfly.runToPosition(pose1_2.getX(), pose1_2.getY(), pose1_2.getHeading(),
                                .85, .6, -poseEstimate.getY(), poseEstimate.getX(),
                                poseEstimate.getHeading());


//                        if (myTimer.time() > 2200)
//                            robot.intakeOuttake.arm.setAligner(true);

                        robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);

                        //robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
                        //robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);

                        if (robot.butterfly.positionReached || myTimer.time() > 3000) {
                            myTimer.reset();
                            state = State.POSITION;
                        }
                    }

                    if (myTimer.time() > 1000)
                        robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);

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
                    //robot.intakeOuttake.arm.aligner.setPwmDisable();

                    robot.butterfly.runToPosition(pose2.getX(), pose2.getY(), pose2.getHeading(),
                            .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
                            poseEstimate.getHeading(), true);

                    double y_difference = junction.getY() - poseEstimate.getX();
                    double x_difference = junction.getX() + poseEstimate.getY();
                    double theta = Math.atan2(y_difference, x_difference) - Math.PI / 2;

                    double turretTargetRad = (Angle.normDelta((back ? theta + Math.PI : theta) - poseEstimate.getHeading()));

                    if (Math.abs(turretTargetRad) > Math.PI / 2) {
                        back = false;
                        if (turretTargetRad < 0)
                            turretTargetRad += Math.PI;
                        else
                            turretTargetRad -= Math.PI;
                    }

                    if (back)
                        distance = Math.hypot(x_difference, y_difference) * 25.4 - 300;
                    else {
                        distance = -(Math.hypot(x_difference, y_difference) * 25.4) + 250;
                    }

                    switch (score) {
                        case EXTEND:
                            //robot.intakeOuttake.arm.claw.setPwmEnable();

                            junction = stack;

                            targetPos = (6 - cycles) * coneOffset + webcamOffset;

                            robot.intakeOuttake.arm.setAligner(true);

                            double adjustment = myTimer.time();

                            if (cycles < 2) {
//                                if (myTimer.time() < 400) {
//                                    robot.intakeOuttake.horizontal.setTarget(distance + 35);
//                                } else {
                                    if (myTimer.time() < 700)
                                        robot.intakeOuttake.horizontal.setTarget(distance - 40 + (700 - adjustment) / 10.0);
                                    else
                                        robot.intakeOuttake.horizontal.setTarget(distance - 70);
                                //}
                            } else {
                                if (myTimer.time() < 700)
                                    robot.intakeOuttake.horizontal.setTarget(distance - 40 + (700 - adjustment) / 10.0);
                                else
                                    robot.intakeOuttake.horizontal.setTarget(distance - 70);
                            }

                            if (cycles < 0) {
                                if (myTimer.time() > 0 && myTimer.time() < 300)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(300, myTimer.time(), 300, (ARM_REST + ARM_INTAKE) / 2, ARM_INTAKE - .06));
                                else if (myTimer.time() >= 300)
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE + .083);
                            } else if (cycles < 2) {
                                if (myTimer.time() > 0 && myTimer.time() < 300)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(300, myTimer.time(), 300, (ARM_REST + ARM_INTAKE) / 2, ARM_INTAKE + .004 - .06));
                                else if (myTimer.time() >= 300)
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE + .007 + .08);
                            } else {
                                if (myTimer.time() > 0 && myTimer.time() < 300)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(300, myTimer.time(), 300, (ARM_REST + ARM_INTAKE) / 2, ARM_INTAKE + .01 - .06));
                                else if (myTimer.time() >= 300)
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE + .011 + .08);
                            }

                            if (myTimer.time() > 100) {
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                if (myTimer.time() > 200 && myTimer.time() < 740) {
                                    robot.intakeOuttake.arm.claw.setPosition(CLAW_AUTO);
                                }
                            }

                            if (myTimer.time() > 750) {
                                //robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                                myTimer.reset();
                                reached = true;
                                score = Score.GRAB;
                            }

                            turret = -Turret.radiansToTicks(turretTargetRad);
                            robot.intakeOuttake.horizontal.update();

                            break;
                        case GRAB:

                            robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                            robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

//                            if (myTimer.time() > 375)
//                                robot.intakeOuttake.arm.claw.setPwmDisable();
//                            else
//                                robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                            robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                            if (myTimer.time() > 250 && reached) {
                                targetPos += safeClear;
                                reached = false;
                            }
                            if (myTimer.time() > 500) {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                            }

                            if( myTimer.time() >= 525 && myTimer.time() <= 1025) {
                                robot.intakeOuttake.arm.arm.setPosition((ARM_INTAKE + (ARM_ANGLED - ARM_INTAKE) * (1 - (1025 - myTimer.time()) / 500)));
                            }

                            if (myTimer.time() > 1025) {
                                junction = B3;
                                back = true;
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

                                targetPos = slidesUp;
                                turret = -Turret.radiansToTicks(turretTargetRad);
                                robot.intakeOuttake.arm.arm.setPosition(ARM_ANGLED);
                                firstTime = true;
                                visionTimer.reset();
                                myTimer.reset();
                                score = Score.UP;
                            }

                            turret = -Turret.radiansToTicks(turretTargetRad);

                            break;
                        case UP:
                            junction = B3;

                            if (!firstTime) {
                                robot.intakeOuttake.horizontal.setTarget(distance + 50);
                            }

                            targetPos = slidesUp;

                            robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);

                            if (cycles == -1) {
                                if (myTimer.time() > 400)
                                    robot.intakeOuttake.horizontal.update();
                            } else {
                                if (myTimer.time() > 300)
                                    robot.intakeOuttake.horizontal.update();
                            }
                            turret = -Turret.radiansToTicks(turretTargetRad);

                            if (myTimer.time() > 600) {
                                myTimer.reset();
                                score = Score.EXTENDSCORE;
                            }
                            firstTime = false;

                            break;
                        case EXTENDSCORE:

//                            if (myTimer.time() > 400)
//                                robot.intakeOuttake.arm.claw.setPwmEnable();

                            if (myTimer.time() <= 475)
                                robot.intakeOuttake.horizontal.setTarget(distance + 50);

                            if (myTimer.time() < 525)
                                robot.intakeOuttake.arm.arm.setPosition(ARM_ANGLED);

                            robot.intakeOuttake.arm.setAligner(true);

                            turret = -Turret.radiansToTicks(turretTargetRad);
                            robot.intakeOuttake.horizontal.update();

                            if (myTimer.time() > 525) {
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE);
                                if (myTimer.time() > 700) {
                                    targetPos -= 250;
                                    myTimer.reset();
                                    score = Score.REST;
                                }
                            }

                            break;
                        case REST:
                            junction = stack;

                            if (myTimer.time() < 400)
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE);

                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

                            robot.intakeOuttake.horizontal.forwardRight.setPosition((FORWARD_RIGHT_IN));
                            robot.intakeOuttake.horizontal.forwardLeft.setPosition((FORWARD_LEFT_IN));

                            if (myTimer.time() > 0 && myTimer.time() < 400) {
//                                robot.intakeOuttake.arm.claw.setPwmEnable();
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                            }

                            if (myTimer.time() > 300 && myTimer.time() < 500) {
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(linearProfile(200, myTimer.time(), 500, ((FORWARD_LEFT_OUT + FORWARD_LEFT_IN) / 2 - .085), FORWARD_LEFT_IN));
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition(linearProfile(200, myTimer.time(), 500, ((FORWARD_RIGHT_OUT + FORWARD_RIGHT_IN) / 2 + .085), FORWARD_RIGHT_IN));

                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_SAFE);
//                            } else if (myTimer.time() > 500) {
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition((FORWARD_RIGHT_IN));
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition((FORWARD_LEFT_IN));
                            }

                            if (myTimer.time() > 700)
                                //robot.intakeOuttake.arm.claw.setPwmDisable();
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                            if (myTimer.time() > 650) {
                                turret = -Turret.radiansToTicks(turretTargetRad);
                                targetPos = (4 - cycles) * coneOffset + webcamOffset;
                            }

                            if (myTimer.time() >= 400 && myTimer.time() <= 1000) {
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + (((ARM_REST + ARM_INTAKE) / 2) - ARM_OUTTAKE) * (1 - (1000 - myTimer.time()) / 600));
                            }

                            if (myTimer.time() > 1000) {
                                robot.intakeOuttake.horizontal.forwardRight.setPosition((FORWARD_RIGHT_IN));
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition((FORWARD_LEFT_IN));

                                robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_OUT + BACKWARD_RIGHT_IN) / 2);
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);

                                robot.intakeOuttake.arm.arm.setPosition(ARM_REST);
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                cycles += 1;
                                myTimer.reset();
                                score = Score.EXTEND;
                            }
                            break;
                    }
                    if (cycles == 5) {
                        state = State.PARK;
                        reached = false;
                    }

                    break;

                case PARK:

                    //robot.intakeOuttake.arm.aligner.setPwmEnable();
                    robot.intakeOuttake.arm.setAligner(false);

                    robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                    robot.intakeOuttake.arm.arm.setPosition(ARM_REST);
                    robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                    targetPos = 50;
                    turret = 0;

                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                    switch (tagOfInterest.id) {
                        case 1:
                            robot.butterfly.runToPosition(left.getX(), left.getY(), left.getHeading(),
                                    .85, 1, -poseEstimate.getY(), poseEstimate.getX(),
                                    poseEstimate.getHeading());
                            break;

                        case 3:
                            robot.butterfly.runToPosition(right.getX(), right.getY(), right.getHeading(),
                                    .85, 1, -poseEstimate.getY(), poseEstimate.getX(),
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
