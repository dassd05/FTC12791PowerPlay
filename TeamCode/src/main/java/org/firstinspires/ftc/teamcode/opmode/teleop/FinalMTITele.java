package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.JUNCTIONS.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.ARM_ANGLED_TELE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.subsystem.vision.JunctionDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.Target;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@TeleOp(name = "FinalMTITele", group = "0")
public class FinalMTITele extends LinearOpMode {

    public static double P = .007, I = 7e-11 , D = 400;

    int slidesTargetPos = 0;
    int turretTarget = 0;

    int driverSlides = 0;
    int driverTurret = 0;
    float driverArm = 0;
    double horizontalDriver = 0.0;

    public boolean back = true;
    boolean aligning = false;

    public double distance = 0;

    public Vector2d junction = C2;

    @Override
    public void runOpMode() {

        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime FSMTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime visionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Robot robot = new Robot(hardwareMap, telemetry);
        telemetry = robot.telemetry;

        JunctionDetectionPipeline pipeline = new JunctionDetectionPipeline();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
        OpenCvCamera cvCamera = OpenCvCameraFactory.getInstance().createWebcam(
                webcamName,
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        cvCamera.setPipeline(pipeline);
        cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cvCamera.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(cvCamera, 60);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Warning: " + webcamName + " could not be opened. Error Code: " + errorCode);
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.setPoseEstimate(PoseStorage.currentPose);

        JustPressed justPressed1 = new JustPressed(gamepad1);
        JustPressed justPressed2 = new JustPressed(gamepad2);

        double totalError = 0.0;
        double lastError = 0.0;

        int slidesTop = 682 - 50;
        int slidesMiddle = 302 - 50;
        int safe = 50 - 50;

        int clear = 300;
        //TODO: fix

        boolean deployed = false;

        boolean firstTime = true;

        boolean cap = false;

        int autoIntake = 0;
        boolean waitTurret = false;

        double backLeftIntaking = Range.scale(.65, 0, 1, BACKWARD_LEFT_IN, BACKWARD_LEFT_OUT);
        double backRightIntaking = Range.scale(.65, 0, 1, BACKWARD_RIGHT_IN, BACKWARD_RIGHT_OUT);

        myState = State.REST;

        boolean armNeutral = true;
        boolean linkageAuto = false;

        boolean hdistance = false;

        boolean turretStill = false;
        boolean visionCorrection = true;
        boolean visionCorrected = false;

        boolean b2 = false;

        robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
        robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

        //robot.intakeOuttake.arm.arm.setPosition(ARM_REST);

        while (opModeInInit()) {
            robot.update();
        }

        waitForStart();

        myTimer.reset();
        FSMTimer.reset();

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {

            if (gamepad2.left_bumper && justPressed2.right_bumper() || justPressed2.left_bumper() && gamepad2.right_bumper)
                drive.setPoseEstimate(new Pose2d(0, -24.25, Math.toRadians(180)));

            Pose2d poseEstimate = drive.getPoseEstimate();

            b2 = (Math.hypot((D2.getY() - poseEstimate.getX()), (D2.getX() + poseEstimate.getY())) < (Math.hypot((B2.getY() - poseEstimate.getX()), (B2.getX() + poseEstimate.getY()))));


//            if (justPressed1.x()) robot.butterfly.setState(Butterfly.State.MECANUM);
            if (justPressed1.b() && !gamepad1.start)
                robot.butterfly.setState(robot.butterfly.getState() != Butterfly.State.MECANUM ? Butterfly.State.MECANUM : Butterfly.State.BAD_TRACTION);
//                robot.butterfly.setState(Butterfly.State.STANDSTILL); // todo make the robot standstill when we're scoring

            double[] fields = {-Math.cbrt(gamepad1.left_stick_y), Math.cbrt(gamepad1.left_stick_x), Math.cbrt(gamepad1.right_stick_x)};
            if (gamepad1.right_bumper || gamepad1.left_stick_button || gamepad1.right_stick_button)
                fields = new double[]{fields[0] * .35, fields[1] * .35, fields[2] * .5};
            robot.butterfly.drive(fields[0], fields[1], fields[2]);


            if (justPressed1.y())
                armNeutral = !armNeutral;
            if (justPressed2.a())
                linkageAuto = !linkageAuto;

            switch (myState) {
                case REST:
                    if (linkageAuto)
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                    else
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                    if (armNeutral)
                        robot.intakeOuttake.arm.arm.setPosition(ARM_REST_TELE);
                    else {
                        if (linkageAuto)
                            robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE_TELE);
                        else
                            robot.intakeOuttake.arm.arm.setPosition((ARM_INTAKE_TELE + ARM_REST_TELE) / 2);
                    }

                    robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);


                    if (linkageAuto) {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);
                    } else {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    }
                    robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                    slidesTargetPos = safe;
                    turretTarget = 0;

                    robot.intakeOuttake.arm.setAligner(linkageAuto);

                    if (justPressed1.a()) {
                        driverArm = 0;
                        driverSlides = 0;
                        driverTurret = 0;
                        armNeutral = false;
                        FSMTimer.reset();
                        myState = State.INTAKE_AUTOMATED;
                    }
                    break;
                case INTAKE_AUTOMATED:
                    if (gamepad2.left_bumper) autoIntake = 1;
                    if (gamepad2.right_bumper) autoIntake = 2;

                    slidesTargetPos = safe;

                    if (FSMTimer.time() > 150) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                        robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE_TELE - driverArm);

                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                    }

                    robot.intakeOuttake.arm.setAligner(true);

                    if (autoIntake == 0) {
                        if (FSMTimer.time() < 500) {
                            if (linkageAuto) {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(500, FSMTimer.time(), 500, (BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2, backLeftIntaking));
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(500, FSMTimer.time(), 500, (BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2, backRightIntaking));
                            } else {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(500, FSMTimer.time(), 500, BACKWARD_LEFT_IN, backLeftIntaking));
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(500, FSMTimer.time(), 500, BACKWARD_RIGHT_IN, backRightIntaking));
                            }
                        } else {
                            if (horizontalDriver < 0) {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(backLeftIntaking - horizontalDriver);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(backRightIntaking + horizontalDriver);
                            } else {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(backLeftIntaking + horizontalDriver);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(backRightIntaking - horizontalDriver);
                            }
                        }
                    } else {
                        Vector2d target = autoIntake == 1 ? new Vector2d() : new Vector2d();
                        double distance = Math.max(poseEstimate.vec().distTo(target) - 2, 0);
                        robot.intakeOuttake.horizontal.setTarget(-distance * 25.4);
                    }
                    robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                    turretTarget = 0;

                    if (justPressed1.a()) {
                        driverArm = 0;
                        driverSlides = 0;
                        driverTurret = 0;
                        armNeutral = false;
                        linkageAuto = true;
                        FSMTimer.reset();
                        myState = State.SCORE_PREP;
                    }

                    // cone placement odo adjust
//                    if (gamepad2.left_bumper && justPressed2.right_bumper() || justPressed2.left_bumper() && gamepad2.right_bumper) {
//                        double turTheta = Turret.ticksToRadians(robot.turret.pos);
//                        double aiming = poseEstimate.getHeading() + turTheta;
//                        double extension = robot.intakeOuttake.horizontal.getPosition();
//                        drive.setPoseEstimate(new Pose2d(0, -24.25, Math.toRadians(180)));
//                    }

                    break;

                case SCORE_PREP:
                    if (FSMTimer.time() < 950 && FSMTimer.time() > 350) {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(600, FSMTimer.time(), 950, backLeftIntaking, BACKWARD_LEFT_IN));
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(600, FSMTimer.time(), 950, backRightIntaking, BACKWARD_RIGHT_IN));
                    }
                    robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                    if (FSMTimer.time() > 425 && FSMTimer.time() < 925)
                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(500, FSMTimer.time(), 925, ARM_INTAKE_TELE, (ARM_REST_TELE)));
                    if (FSMTimer.time() > 925)
                        robot.intakeOuttake.arm.arm.setPosition(ARM_REST_TELE);

                    if (FSMTimer.time() > 750) {
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                    }

                    robot.intakeOuttake.arm.setAligner(!(FSMTimer.time() > 1150));

                    boolean reset = false;

                    if (gamepad1.dpad_down) {
                        reset = true;
                        mySlides = Slides.LOW;
                        myState = State.UP;
                    }
                    if (gamepad1.dpad_up) {
                        reset = true;
                        mySlides = Slides.HIGH;
                        myState = State.UP;
                    }
                    if (gamepad1.dpad_right || gamepad1.dpad_left) {
                        reset = true;
                        mySlides = Slides.MIDDLE;
                        myState = State.UP;
                    }

                    if (gamepad2.y) {
                        reset = true;
                        back = true;
                        mySlides = Slides.HIGH;
                        if (justPressed2.dpad_down()) {
                            junction = C2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (justPressed2.dpad_left()) {
                            junction = B3;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (justPressed2.dpad_right()) {
                            junction = D3;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (justPressed2.dpad_up()) {
                            junction = C4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    } else if (gamepad2.x) {
                        reset = true;
                        back = true;
                        mySlides = Slides.MIDDLE;
                        if (gamepad2.dpad_down && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_down()) {
                            junction = B2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_down && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_down()) {
                            junction = D2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                            junction = B4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                            junction = D4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    } else if (gamepad2.a) {
                        reset = true;
                        back = true;
                        mySlides = Slides.LOW;
//                        if (justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_down()) {
                        if (justPressed2.dpad_left()) {
                            junction = A2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_down && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_down()) {
                        } else if (justPressed2.dpad_down()) {
                            junction = B1;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                        } else if (justPressed2.dpad_up()) {
                            junction = D5;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                        } else if (justPressed2.dpad_right()) {
                            junction = E4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    } else if (gamepad2.b) {
                        reset = true;
                        back = true;
                        mySlides = Slides.LOW;
//                        if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                        if (justPressed2.dpad_left()) {
                            junction = A4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                        } else if (justPressed2.dpad_right()) {
                            junction = E2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_down && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_down()) {
                        } else if (justPressed2.dpad_down()) {
                            junction = D2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_down && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_down()) {
                        } else if (justPressed2.dpad_up()) {
                            junction = B5;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    }

                    if (reset) {
                        driverArm = 0;
                        driverSlides = 0;
                        driverTurret = 0;
                        armNeutral = false;
                        firstTime = true;
                        robot.intakeOuttake.arm.setAligner(true);
                    }

                    break;
                case UP:

                    if (firstTime) {
                        aligning = true;
                        firstTime = false;
                    }

                    if(justPressed1.x())
                        aligning = !aligning;

                    robot.intakeOuttake.arm.setAligner(aligning);

                    switch (mySlides) {
                        case LOW:
                            slidesTargetPos = safe;

                            if (FSMTimer.time() < 400)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, FSMTimer.time(), 400, ARM_REST_TELE, ARM_ANGLED_TELE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE_TELE + driverArm);
                            break;
                        case MIDDLE:
                            slidesTargetPos = slidesMiddle;

                            if (FSMTimer.time() < 700)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(700, FSMTimer.time(), 800, ARM_REST_TELE, ARM_ANGLED_TELE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_ANGLED_TELE + driverArm);
                            break;
                        case HIGH:
                            slidesTargetPos = slidesTop;

                            if (FSMTimer.time() < 1000)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(1000, FSMTimer.time(), 800, ARM_REST_TELE, ARM_ANGLED_TELE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_ANGLED_TELE + driverArm);
                            break;
                    }

                    if (gamepad1.dpad_down)
                        mySlides = Slides.LOW;
                    if (gamepad1.dpad_up)
                        mySlides = Slides.HIGH;
                    if (gamepad1.dpad_right || gamepad1.dpad_left)
                        mySlides = Slides.MIDDLE;

                    if (horizontalDriver < 0) {
                        robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                        robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN - horizontalDriver);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN + horizontalDriver);
                    } else if (horizontalDriver > 0) {
                        robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN + horizontalDriver);
                        robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN - horizontalDriver);
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    } else {
                        robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                        robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    }

                    if (justPressed1.x())
                        robot.intakeOuttake.arm.setAligner(!robot.intakeOuttake.arm.isAligning());

                    if (gamepad1.left_bumper) {
                        firstTime = true;
                        FSMTimer.reset();
//                        if (mySlides != Slides.LOW)
                        myState = State.SCORE;
                        cap = false;
//                        else
//                            myState = State.DEEXTEND;
                    }
                    linkageAuto = true;

                    if (gamepad2.right_bumper /*&& gamepad2.dpad_down*/) {
                        driverTurret = 0;
                        horizontalDriver = 0;
                        driverArm = 0;
                        driverSlides = 0;
                        FSMTimer.reset();
                        myState = State.UNDO;
                    }

                    break;
                case UP_AUTOMATIC:
                    hdistance = true;

                    if (firstTime) {
                        aligning = true;

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
                            distance = -(Math.hypot(x_difference, y_difference) * 25.4) + 260;
                        }

                        turretTarget = -Turret.radiansToTicks(turretTargetRad);

                        robot.turret.setTurretTargetPosition(turretTarget + driverTurret);
                        robot.update();

                        firstTime = false;
                        if (b2)
                            visionCorrection = junction != D2 && junction != C2;
                        else
                            visionCorrection = junction != B2 && junction != C2;
                        visionCorrected = false;
                        turretStill = false;
                        visionTimer.reset();
                    }


                    if (visionCorrection && !turretStill && Turret.getWithin() //Math.abs(robot.turret.quadratureEncoder.getCurrentPosition() + turretTarget) < 50
                            && Math.abs(robot.turret.quadratureEncoder.getRawVelocity()) < 5) {
                        visionTimer.reset();
                        turretStill = true;
                    }
                    List<Target> targets;
                    if (b2) {
                        if (visionCorrection && turretStill && visionTimer.time() > (junction == D2 || junction == C2 ? 200 : 200)
                                //&& Math.abs(robot.turret.quadratureEncoder.getCurrentPosition() - turretTarget) < 10
                                && Math.abs(robot.turret.quadratureEncoder.getRawVelocity()) < 5 && (targets = pipeline.getJunctionsTeleop()).size() > 0) {
                            visionCorrection = false;
                            visionCorrected = true;
                            turretStill = false;
                            turretTarget = robot.turret.pos - Turret.radiansToTicks(Math.toRadians(
                                    targets.size() == 1 ? targets.get(0).offset : targets.get(0).rect.width > targets.get(1).rect.width ? targets.get(0).offset : targets.get(1).offset));
                        }
                    } else {
                        if (visionCorrection && turretStill && visionTimer.time() > (junction == B2 || junction == C2 ? 200 : 200)
                                //&& Math.abs(robot.turret.quadratureEncoder.getCurrentPosition() - turretTarget) < 10
                                && Math.abs(robot.turret.quadratureEncoder.getRawVelocity()) < 5 && (targets = pipeline.getJunctionsTeleop()).size() > 0) {
                            visionCorrection = false;
                            visionCorrected = true;
                            turretStill = false;
                            turretTarget = robot.turret.pos - Turret.radiansToTicks(Math.toRadians(
                                    targets.size() == 1 ? targets.get(0).offset : targets.get(0).rect.width > targets.get(1).rect.width ? targets.get(0).offset : targets.get(1).offset));
                        }
                    }

//                    if (distance > 250) {
//                        if (visionTimer.time() > 350 && visionTimer.time() < distance/2+250)
//                            robot.intakeOuttake.arm.aligner.setPosition(MECH_ALIGN_BACK);
//                    }

//                    if (back && FSMTimer.milliseconds() > Math.abs(distance))
//                        robot.intakeOuttake.arm.setAligner(true);

                    if(justPressed1.x())
                        aligning = !aligning;

                    robot.intakeOuttake.arm.setAligner(aligning);

                    if (!firstTime) {
                        if (justPressed2.left_bumper()) {
                            cap = true;
                            FSMTimer.reset();
                        }
                    }

                    switch (mySlides) {
                        case LOW:
                            if (back) {
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                                if (FSMTimer.time() < 400)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, FSMTimer.time(), 400, ARM_REST_TELE, ARM_ANGLED_TELE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE_TELE + .02 + driverArm);
                            }  else {
                                slidesTargetPos = clear;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                if (FSMTimer.time() < 400)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, FSMTimer.time(), 400, ARM_REST_TELE, ARM_INTAKE_TELE - .08));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE_TELE + .02 + driverArm);
                            }

                            if (cap)
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_CAP);
                            break;
                        case MIDDLE:
                            if (back) {
                                slidesTargetPos = slidesMiddle;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                                if (FSMTimer.time() < 700)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(700, FSMTimer.time(), 800, ARM_REST_TELE, ARM_ANGLED_TELE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_ANGLED_TELE + driverArm);
                            } else {
                                slidesTargetPos = slidesMiddle + clear;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                if (FSMTimer.time() < 700)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(700, FSMTimer.time(), 800, ARM_REST_TELE, ARM_INTAKE_TELE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE_TELE + driverArm);
                            }
                            break;
                        case HIGH:
                            if (back) {
                                slidesTargetPos = slidesTop;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                                if (FSMTimer.time() < 1000)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(1000, FSMTimer.time(), 800, ARM_REST_TELE, ARM_ANGLED_TELE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_ANGLED_TELE + driverArm);
                            } else {
                                slidesTargetPos = slidesTop + clear;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                if (FSMTimer.time() < 1000)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(1000, FSMTimer.time(), 800, ARM_REST_TELE, ARM_INTAKE_TELE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE_TELE + driverArm);
                            }
                            break;
                    }

                    if (Turret.getWithin() && Math.abs((slidesTargetPos + driverSlides) - robot.vertical.v2.getCurrentPosition()) < 100 /*&& (junction != B2 && junction != C2 || visionCorrected)*/) {
                        robot.intakeOuttake.horizontal.setTarget(distance + horizontalDriver);
                        robot.intakeOuttake.horizontal.update();
                    }

                    if (gamepad1.left_bumper) {
                        firstTime = true;
                        FSMTimer.reset();
//                        if (mySlides != Slides.LOW)
                        myState = State.SCORE;
                        cap = false;
//                        else
//                            myState = State.DEEXTEND;
                    }

                    if (gamepad2.right_bumper /*&& gamepad2.dpad_down*/) {
                        driverTurret = 0;
                        horizontalDriver = 0;
                        distance = 0;
                        driverArm = 0;
                        driverSlides = 0;
                        FSMTimer.reset();
                        myState = State.UNDO;
                    }

                    linkageAuto = false;
                    break;
                case SCORE:
//                    if (firstTime) {
//                        double turTheta = Turret.ticksToRadians(robot.turret.pos);
//                        double aiming = poseEstimate.getHeading() + turTheta;
//                        double extension = robot.intakeOuttake.horizontal.getPosition();
//                        double xOffset = Math.cos(aiming) * extension;
//                        double yOffset = Math.sin(aiming) * extension;
//                        drive.setPoseEstimate(new Pose2d(junction.getX() - xOffset, junction.getY() - yOffset, poseEstimate.getHeading()));
//                        // todo heading estimate bad
//                    }

                    if (gamepad2.right_bumper /*&& gamepad2.dpad_down*/) {
                        driverTurret = 0;
                        horizontalDriver = 0;
                        driverArm = 0;
                        driverSlides = 0;
                        FSMTimer.reset();
                        myState = State.UNDO;
                    }

                    if (!firstTime) {
                        if (justPressed2.left_bumper()) {
                            cap = true;
                            FSMTimer.reset();
                        }
                    }

                    if (mySlides == Slides.LOW) {
                        if (cap) {
                            robot.intakeOuttake.arm.claw.setPosition(CLAW_CAP);
                        }
                        else
                            robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                    }
                    else {
                        if (firstTime)
//                            slidesTargetPos -= 500;
                            robot.intakeOuttake.arm.arm.setPosition(robot.intakeOuttake.arm.arm.getPosition() - .08);

                        if (cap) {
                            if (FSMTimer.time() > 200)
                                robot.intakeOuttake.arm.arm.setPosition(ARM_REST_TELE);
                            robot.intakeOuttake.arm.claw.setPosition(CLAW_CAP);
                        }
                    }
                    if (justPressed1.dpad_up())
//                        slidesTargetPos += 500;
                        robot.intakeOuttake.arm.arm.setPosition(robot.intakeOuttake.arm.arm.getPosition() + .08);
                    if (justPressed1.left_bumper() && !firstTime)
//                        slidesTargetPos -= 500;
                        robot.intakeOuttake.arm.arm.setPosition(robot.intakeOuttake.arm.arm.getPosition() - .08);

                    if (hdistance) {
                        robot.intakeOuttake.horizontal.setTarget(distance + horizontalDriver);
                        robot.intakeOuttake.horizontal.update();
                    }

                    if (justPressed1.x())
                        robot.intakeOuttake.arm.setAligner(!robot.intakeOuttake.arm.isAligning());

                    firstTime = false;

                    if (gamepad1.dpad_down) {
                        FSMTimer.reset();
                        driverTurret = 0;
                        driverSlides = 0;
                        driverArm = 0;
                        hdistance = false;
                        myState = State.DEEXTEND;
                    }

//                    if (cap) {
//                        if (justPressed1.left_bumper()) {
//                            robot.intakeOuttake.arm.claw.setPosition(CLAW_CAP);
//                        } else {
//                            if (robot.intakeOuttake.arm.claw.getPosition() == CLAW_CAP) {
//                                if (myTimer.time() > 500) {
//                                    robot.intakeOuttake.arm.arm.setPosition(ARM_REST);
//                                    firstTime = true;
//                                    FSMTimer.reset();
//                                    myState = State.UP;
//                                }
//                            } else {
//                                FSMTimer.reset();
//                            }
//                        }
//                    }

                    break;
                case DEEXTEND:
                    slidesTargetPos = safe;
//                    if (robot.intakeOuttake.horizontal.forwardLeft.getPosition() > (FORWARD_LEFT_IN * 3 + FORWARD_LEFT_OUT) / 4)
//                        turretTarget = 0;
                    if (Math.abs(robot.intakeOuttake.horizontal.getPosition()) > 300)
                        waitTurret = true;
                    if (waitTurret && FSMTimer.time() > 750)
                        turretTarget = 0;

                    if (FSMTimer.time() < 500)
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);

                    if (FSMTimer.time() < 750 && FSMTimer.time() > 500) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_SAFE);
                    } else {
//                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
//                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                    }

                    if (robot.intakeOuttake.horizontal.forwardLeft.getPosition() > (FORWARD_LEFT_IN * 3 + FORWARD_LEFT_OUT) / 4) {
                        if (linkageAuto) {
                            if (FSMTimer.time() < 750)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(750, FSMTimer.time(), 750, ARM_ANGLED_TELE, ARM_INTAKE_TELE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE_TELE);
                        } else {
                            if (FSMTimer.time() < 750)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(750, FSMTimer.time(), 750, ARM_ANGLED_TELE, ARM_REST_TELE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_REST_TELE);
                        }
                    }

                    if (linkageAuto) {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);
                    } else {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    }
                    robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                    robot.intakeOuttake.arm.setAligner(true);

                    if (FSMTimer.time() > 750) {
                        driverTurret = 0;
                        horizontalDriver = 0;
                        driverArm = 0;
                        driverSlides = 0;
                        waitTurret = false;
                        FSMTimer.reset();
                        myState = State.REST;
                    }
                    break;
                case UNDO:

                    if (FSMTimer.time() > 350)
                        slidesTargetPos = safe;

                    robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                    robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);

                    if (FSMTimer.time() > 750)
                        turretTarget = 0;

                    if (FSMTimer.time() < 750)
                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(750, FSMTimer.time(), 750, ARM_ANGLED_TELE, ARM_REST_TELE));
                    else
                        robot.intakeOuttake.arm.arm.setPosition(ARM_REST_TELE);

                    robot.intakeOuttake.arm.setAligner(!(FSMTimer.time() > 950));


                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

                    robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                    reset = false;

                    if (gamepad1.dpad_down) {
                        reset = true;
                        mySlides = Slides.LOW;
                        myState = State.UP;
                    }
                    if (gamepad1.dpad_up) {
                        reset = true;
                        mySlides = Slides.HIGH;
                        myState = State.UP;
                    }
                    if (gamepad1.dpad_right || gamepad1.dpad_left) {
                        reset = true;
                        mySlides = Slides.MIDDLE;
                        myState = State.UP;
                    }

                    if (gamepad2.y) {
                        reset = true;
                        back = true;
                        mySlides = Slides.HIGH;
                        if (justPressed2.dpad_down()) {
                            junction = C2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (justPressed2.dpad_left()) {
                            junction = B3;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (justPressed2.dpad_right()) {
                            junction = D3;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (justPressed2.dpad_up()) {
                            junction = C4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    } else if (gamepad2.x) {
                        reset = true;
                        back = true;
                        mySlides = Slides.MIDDLE;
                        if (gamepad2.dpad_down && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_down()) {
                            junction = B2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_down && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_down()) {
                            junction = D2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                            junction = B4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                            junction = D4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    } else if (gamepad2.a) {
                        reset = true;
                        back = true;
                        mySlides = Slides.LOW;
//                        if (justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_down()) {
                        if (justPressed2.dpad_left()) {
                            junction = A2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_down && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_down()) {
                        } else if (justPressed2.dpad_down()) {
                            junction = B1;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                        } else if (justPressed2.dpad_up()) {
                            junction = D5;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                        } else if (justPressed2.dpad_right()) {
                            junction = E4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    } else if (gamepad2.b) {
                        reset = true;
                        back = true;
                        mySlides = Slides.LOW;
//                        if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                        if (justPressed2.dpad_left()) {
                            junction = A4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                        } else if (justPressed2.dpad_right()) {
                            junction = E2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_down && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_down()) {
                        } else if (justPressed2.dpad_down()) {
                            junction = D2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
//                        } else if (gamepad2.dpad_down && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_down()) {
                        } else if (justPressed2.dpad_up()) {
                            junction = B5;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    }

                    if (reset) {
                        driverArm = 0;
                        driverSlides = 0;
                        driverTurret = 0;
                        armNeutral = false;
                        firstTime = true;
                        robot.intakeOuttake.arm.setAligner(true);
                    }

//                    if (FSMTimer.time() > 750) {
//                        driverTurret = 0;
//                        horizontalDriver = 0;
//                        driverArm = 0;
//                        driverSlides = 0;
//                        waitTurret = false;
//                        FSMTimer.reset();
//                        myState = State.REST;
//                    }
                    break;
            }

            long timeDelta = System.nanoTime() - lastTime;
            driverTurret += (int) (cube(gamepad2.right_stick_x) * (timeDelta / 1e9) * 700 / Math.max(Math.abs(robot.intakeOuttake.horizontal.getPosition()), 150) * 150);
            if (back)
                driverArm -= cube(gamepad2.left_stick_y) * timeDelta / 1e9 * .16;
            else
                driverArm += cube(gamepad2.left_stick_y) * timeDelta / 1e9 * .16;
            if (myState != State.SCORE_PREP) {
                if (myState == State.UP_AUTOMATIC || myState == State.SCORE)
                    horizontalDriver += timeDelta / 1e9 * (gamepad1.right_trigger > .5 ? 270 : gamepad1.left_trigger > .5 ? -270 : 0);
                else
                    horizontalDriver += timeDelta / 1e9 * .175 * (gamepad1.right_trigger > .5 ? 1.3 : gamepad1.left_trigger > .5 ? -1.3 : 0);
            }
            driverSlides += timeDelta / 1e9 * 150 * (gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);
            if (myState == State.UP_AUTOMATIC || myState == State.UP)
                robot.intakeOuttake.arm.adjustAlignerDegrees(timeDelta / 1e9 * 10 * (-gamepad2.left_trigger + gamepad2.right_trigger));

            long time = System.nanoTime();
            double error = (slidesTargetPos + driverSlides) - robot.vertical.v2.getCurrentPosition();
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

            robot.turret.setTurretTargetPosition(turretTarget + driverTurret);

//            if (robot.intakeOuttake.horizontal.getPosition() < 0)
//                robot.intakeOuttake.arm.setAligner(true);

//
//            telemetry.addData("x difference", x_difference);
//            telemetry.addData("y difference", y_difference);
//            telemetry.addData("theta", theta);
//            telemetry.addData("rr heading", poseEstimate.getHeading());
//            telemetry.addData("turret radians", turretTargetRad);
//            telemetry.addData("turret target", ttarget);
//            telemetry.addData("distaance", distance);
            telemetry.addData("State", myState);
            telemetry.addData("Pose x*", -poseEstimate.getY());
            telemetry.addData("Pose y*", poseEstimate.getX());
            telemetry.addData("Pose Heading", poseEstimate.getHeading());
            telemetry.addData("horizontal distance (mm)", robot.intakeOuttake.horizontal.getPosition());
            telemetry.addData("Turret Target", turretTarget);
            telemetry.addData("Turret Position", robot.intakeOuttake.turret.quadratureEncoder.getCurrentPosition());
            telemetry.addData("Turret Velocity", robot.intakeOuttake.turret.quadratureEncoder.getRawVelocity());
            telemetry.addData("TurretStill", turretStill);
            telemetry.addData("VisionCorrection", visionCorrection);
            Target jjjjj = pipeline.getJunctionTeleop();
            telemetry.addData("Junction", jjjjj == null ? "null" : jjjjj.rect.toString());

            drive.update();

            telemetry.update();
            robot.update();
            justPressed1.update();
            justPressed2.update();
        }
    }

    public enum State {
        UP,
        UP_AUTOMATIC,
        REST,
        INTAKE_AUTOMATED,
        INTAKE_DRIVER,
        SCORE_PREP,
        SCORE,
        DEEXTEND,
        UNDO
    }
    public State myState;

    public enum Slides {
        LOW,
        MIDDLE,
        HIGH
    }
    public Slides mySlides;

    public double linearProfile(double totalTime, double currentTime, double endTime, double initial, double fin) {
        return (initial + (fin - initial) * (1 - (endTime - currentTime) / totalTime));
    }

    public static double cube(double x) {
        return x*x*x;
    }
}
