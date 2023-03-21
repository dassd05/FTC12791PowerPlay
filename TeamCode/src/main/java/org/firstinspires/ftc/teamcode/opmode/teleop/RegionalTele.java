package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.JUNCTIONS.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name = "RegionalTele", group = "0")
public class RegionalTele extends LinearOpMode {

    public static double P = .0035, I = .000000000001 , D = 25;

    int slidesTargetPos = 0;
    int turretTarget = 0;

    int driverSlides = 0;
    int driverTurret = 0;
    float driverArm = 0;
    double horizontalDriver = 0.0;

    public boolean back = true;

    public double distance = 0;

    public Vector2d junction = C2;

    @Override
    public void runOpMode() {

        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime FSMTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Robot robot = new Robot(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.setPoseEstimate(PoseStorage.currentPose);

        JustPressed justPressed1 = new JustPressed(gamepad1);
        JustPressed justPressed2 = new JustPressed(gamepad2);

        double totalError = 0.0;
        double lastError = 0.0;

        int slidesTop = 1530;
        int slidesMiddle = 870;
        int safe = 125;

        int clear = 920;

        boolean deployed = false;

        boolean firstTime = true;

        double backLeftIntaking = Range.scale(.65, 0, 1, BACKWARD_LEFT_IN, BACKWARD_LEFT_OUT);
        double backRightIntaking = Range.scale(.65, 0, 1, BACKWARD_RIGHT_IN, BACKWARD_RIGHT_OUT);

        myState = State.REST;

        boolean armNeutral = true;
        boolean linkageAuto = false;

        robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
        robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

        robot.intakeOuttake.arm.arm.setPosition(ARM_REST);

        waitForStart();

        myTimer.reset();
        FSMTimer.reset();

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {

            if (gamepad2.left_bumper && justPressed2.right_bumper() || justPressed2.left_bumper() && gamepad2.right_bumper)
                drive.setPoseEstimate(new Pose2d(0, -24.25, Math.toRadians(180)));

            Pose2d poseEstimate = drive.getPoseEstimate();

            if (justPressed1.x()) robot.butterfly.setState(Butterfly.State.MECANUM);
            if (justPressed1.b() && !gamepad1.start)
                robot.butterfly.setState(Butterfly.State.STANDSTILL);

            double[] fields = {-Math.cbrt(gamepad1.left_stick_y), Math.cbrt(gamepad1.left_stick_x), Math.cbrt(gamepad1.right_stick_x)};
            if (gamepad1.right_bumper)
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
                        robot.intakeOuttake.arm.arm.setPosition(ARM_REST);
                    else {
                        if (linkageAuto)
                            robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE);
                        else
                            robot.intakeOuttake.arm.arm.setPosition((ARM_INTAKE + ARM_REST) / 2);
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
                    slidesTargetPos = safe;

                    robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE);

                    robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);

                    if (FSMTimer.time() < 700) {
                        if (linkageAuto) {
                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(700, FSMTimer.time(), 700, (BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2, backLeftIntaking));
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(700, FSMTimer.time(), 700, (BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2, backRightIntaking));
                        } else {
                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(700, FSMTimer.time(), 700, BACKWARD_LEFT_IN, backLeftIntaking));
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(700, FSMTimer.time(), 700, BACKWARD_RIGHT_IN, backRightIntaking));
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
                    if (FSMTimer.time() < 1200 && FSMTimer.time() > 500) {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(700, FSMTimer.time(), 1200, backLeftIntaking, BACKWARD_LEFT_IN));
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(700, FSMTimer.time(), 1200, backRightIntaking, BACKWARD_RIGHT_IN));
                    }
                    robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                    if (FSMTimer.time() > 675 && FSMTimer.time() < 1175)
                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(500, FSMTimer.time(), 1175, ARM_INTAKE, (ARM_REST)));
                    if (FSMTimer.time() > 1175)
                        robot.intakeOuttake.arm.arm.setPosition(ARM_REST);

                    if (FSMTimer.time() > 1000) {
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                    }

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
                        if (gamepad2.dpad_down && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_down()) {
                            junction = B1;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_down && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_down()) {
                            junction = D1;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                            junction = A2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                            junction = E2;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        }
                    } else if (gamepad2.b) {
                        reset = true;
                        back = true;
                        mySlides = Slides.LOW;
                        if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                            junction = B5;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                            junction = D5;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_left() || gamepad2.dpad_left && justPressed2.dpad_up()) {
                            junction = A4;
                            FSMTimer.reset();
                            myState = State.UP_AUTOMATIC;
                        } else if (gamepad2.dpad_up && justPressed2.dpad_right() || gamepad2.dpad_right && justPressed2.dpad_up()) {
                            junction = E4;
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
                    }

                    break;
                case UP:
                    switch (mySlides) {
                        case LOW:
                            if (FSMTimer.time() < 400)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, FSMTimer.time(), 400, ARM_REST, ARM_OUTTAKE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + driverArm);
                            break;
                        case MIDDLE:
                            slidesTargetPos = slidesMiddle;

                            if (FSMTimer.time() < 700)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(700, FSMTimer.time(), 800, ARM_REST, ARM_OUTTAKE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + driverArm);
                            break;
                        case HIGH:
                            slidesTargetPos = slidesTop;

                            if (FSMTimer.time() < 1000)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(1000, FSMTimer.time(), 800, ARM_REST, ARM_OUTTAKE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + driverArm);
                            break;
                    }

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

                    if (gamepad1.left_bumper) {
                        firstTime = true;
                        FSMTimer.reset();
//                        if (mySlides != Slides.LOW)
                        myState = State.SCORE;
//                        else
//                            myState = State.DEEXTEND;
                    }
                    linkageAuto = true;
                    break;
                case UP_AUTOMATIC:

                    if (firstTime) {
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
                            distance = Math.hypot(x_difference, y_difference) * 25.4 - 400;
                        else {
                            distance = -(Math.hypot(x_difference, y_difference) * 25.4) + 318;
                        }

                        turretTarget = -Turret.radiansToTicks(turretTargetRad);

                        firstTime = false;
                    }


                    switch (mySlides) {
                        case LOW:
                            if (back) {
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                                if (FSMTimer.time() < 400)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, FSMTimer.time(), 400, ARM_REST, ARM_OUTTAKE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + driverArm);
                            }  else {
                                slidesTargetPos = clear;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                if (FSMTimer.time() < 400)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(400, FSMTimer.time(), 400, ARM_REST, ARM_INTAKE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE + driverArm);
                            }
                            break;
                        case MIDDLE:
                            if (back) {
                                slidesTargetPos = slidesMiddle;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                                if (FSMTimer.time() < 700)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(700, FSMTimer.time(), 800, ARM_REST, ARM_OUTTAKE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + driverArm);
                            } else {
                                slidesTargetPos = slidesMiddle + clear;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                if (FSMTimer.time() < 700)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(700, FSMTimer.time(), 800, ARM_REST, ARM_INTAKE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE + driverArm);
                            }
                            break;
                        case HIGH:
                            if (back) {
                                slidesTargetPos = slidesTop;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                                if (FSMTimer.time() < 1000)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(1000, FSMTimer.time(), 800, ARM_REST, ARM_OUTTAKE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + driverArm);
                            } else {
                                slidesTargetPos = slidesTop + clear;
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                if (FSMTimer.time() < 1000)
                                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(1000, FSMTimer.time(), 800, ARM_REST, ARM_INTAKE));
                                else
                                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE + driverArm);
                            }
                            break;
                    }

                    if (Turret.getWithin() && Math.abs((slidesTargetPos + driverSlides) - robot.vertical.v2.getCurrentPosition()) < 150) {
                        robot.intakeOuttake.horizontal.setTarget(distance + horizontalDriver);
                        robot.intakeOuttake.horizontal.update();
                    }

                    if (gamepad1.left_bumper) {
                        firstTime = true;
                        FSMTimer.reset();
//                        if (mySlides != Slides.LOW)
                        myState = State.SCORE;
//                        else
//                            myState = State.DEEXTEND;
                    }
                    linkageAuto = false;
                    break;
                case SCORE:
                    if (firstTime) {
                        double turTheta = Turret.ticksToRadians(robot.turret.pos);
                        double aiming = poseEstimate.getHeading() + turTheta;
                        double extension = robot.intakeOuttake.horizontal.getPosition();
                        double xOffset = Math.cos(aiming) * extension;
                        double yOffset = Math.sin(aiming) * extension;
                        drive.setPoseEstimate(new Pose2d(junction.getX() - xOffset, junction.getY() - yOffset, poseEstimate.getHeading()));
                        // todo heading estimate bad
                    }

                    if (mySlides == Slides.LOW) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                    }
                    else {
                        if (firstTime)
                            slidesTargetPos -= 500;
                    }
                    firstTime = false;

                    if (gamepad1.dpad_down) {
                        FSMTimer.reset();
                        driverTurret = 0;
                        driverSlides = 0;
                        driverArm = 0;
                        myState = State.DEEXTEND;
                    }
                    break;
                case DEEXTEND:
                    slidesTargetPos = safe;
                    if (robot.intakeOuttake.horizontal.forwardLeft.getPosition() > (FORWARD_LEFT_IN * 3 + FORWARD_LEFT_OUT) / 4)
                        turretTarget = 0;

                    if (FSMTimer.time() < 350)
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);

                    if (FSMTimer.time() < 750 && FSMTimer.time() > 350) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_SAFE);
                    } else {
//                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
//                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                    }

                    if (robot.intakeOuttake.horizontal.forwardLeft.getPosition() > (FORWARD_LEFT_IN * 3 + FORWARD_LEFT_OUT) / 4) {
                        if (linkageAuto) {
                            if (FSMTimer.time() < 750)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(750, FSMTimer.time(), 750, ARM_OUTTAKE, ARM_INTAKE));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE);
                        } else {
                            if (FSMTimer.time() < 750)
                                robot.intakeOuttake.arm.arm.setPosition(linearProfile(750, FSMTimer.time(), 750, ARM_OUTTAKE, ARM_REST));
                            else
                                robot.intakeOuttake.arm.arm.setPosition(ARM_REST);
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

                    if (FSMTimer.time() > 750) {
                        driverTurret = 0;
                        horizontalDriver = 0;
                        driverArm = 0;
                        driverSlides = 0;
                        FSMTimer.reset();
                        myState = State.REST;
                    }
                    break;
            }

            long timeDelta = System.nanoTime() - lastTime;
            driverTurret += (int) (cube(gamepad2.right_stick_x) * (timeDelta / 1e9) * 350);
            if (back)
                driverArm -= cube(gamepad2.left_stick_y) * timeDelta / 1e9 * .2;
            else
                driverArm += cube(gamepad2.left_stick_y) * timeDelta / 1e9 * .2;
            if (myState != State.SCORE_PREP) {
                if (myState == State.UP_AUTOMATIC)
                    horizontalDriver += timeDelta / 1e9 * (gamepad1.right_trigger > .5 ? 270 : gamepad1.left_trigger > .5 ? -270 : 0);
                else
                    horizontalDriver += timeDelta / 1e9 * .175 * (gamepad1.right_trigger > .5 ? 1.3 : gamepad1.left_trigger > .5 ? -1.3 : 0);
            }
            driverSlides += timeDelta / 1e9 * 300 * (gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);

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
//
//            telemetry.addData("x difference", x_difference);
//            telemetry.addData("y difference", y_difference);
//            telemetry.addData("theta", theta);
//            telemetry.addData("rr heading", poseEstimate.getHeading());
//            telemetry.addData("turret radians", turretTargetRad);
//            telemetry.addData("turret target", ttarget);
//            telemetry.addData("distaance", distance);

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
        DEEXTEND
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
