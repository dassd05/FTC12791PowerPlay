package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.BACKWARD_LEFT_IN;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.BACKWARD_LEFT_OUT;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.BACKWARD_RIGHT_IN;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.BACKWARD_RIGHT_OUT;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.FORWARD_LEFT_IN;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.FORWARD_LEFT_OUT;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.FORWARD_RIGHT_IN;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.FORWARD_RIGHT_OUT;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.intakeLinkage1In;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.intakeLinkage2In;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.outtakeLinkage1In;
import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.outtakeLinkage2In;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmode.teleop.TestTeleop;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.vision.SignalDetectionPipeline;

@Autonomous (name = "AlternateTourneyAuton", group = "0", preselectTeleOp = "TourneyTele")

public class AlternateTourneyAuton extends LinearOpMode {

    public static double P = .0035, I = .0000000000007, D = 25;

    public double targetPos = 0.0;

    public double coneOffset = 77;
    public double webcamOffset = 15;
    public double safeClear = 440;
    public double slidesUp = 1550;


    public int turret = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SignalDetectionPipeline pipeline = new SignalDetectionPipeline();
        Webcam webcam = new Webcam(hardwareMap, pipeline);

        SignalDetectionPipeline.ParkPosition parkPosition = SignalDetectionPipeline.ParkPosition.MIDDLE;

        Pose2d pose1 = new Pose2d(0,60,0);
        Pose2d pose1_2 = new Pose2d(4,46,45);
        Pose2d pose2 = new Pose2d(-6.2,50.5,90);

        Pose2d left = new Pose2d(-24,50.5,90);
        Pose2d right = new Pose2d(24,50.5,90);
        Pose2d middle = new Pose2d(-2,50.5,90);

        int cycles = 0;

        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeInInit()) {
            //sleep(250);
            parkPosition = pipeline.position;
            telemetry.addData("position", parkPosition);
            if (pipeline.average != null) telemetry.addData("color", pipeline.average.toString());
            telemetry.update();

            robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
            robot.intakeOuttake.arm.arm.setPosition(.693);
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
        myTimer.reset();

        boolean reached = false;

        long lastTime = System.nanoTime();

        waitForStart();

        state = State.FORWARD;
        score = Score.EXTEND;

        robot.butterfly.setState(Butterfly.State.MECANUM);

        while (opModeIsActive()) {

            Pose2d poseEstimate = drive.getPoseEstimate();

            switch (state) {
                case FORWARD:
                    if (!reached) {
                        robot.butterfly.runToPosition(pose1.getX(), pose1.getY(), pose1.getHeading(),
                                .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
                                poseEstimate.getHeading());
                        if (robot.butterfly.positionReached || myTimer.time() > 2000)
                            reached = true;
                    }
                    if (reached) {
                        robot.butterfly.runToPosition(pose1_2.getX(), pose1_2.getY(), pose1_2.getHeading(),
                                .85, .4, -poseEstimate.getY(), poseEstimate.getX(),
                                poseEstimate.getHeading());

                        if (robot.butterfly.positionReached || myTimer.time() > 3000) {
                            myTimer.reset();
                            state = State.POSITION;
                        }
                    }
                    break;

                case POSITION:
                    robot.butterfly.runToPosition(pose2.getX(), pose2.getY(), pose2.getHeading(),
                            .45, 1, -poseEstimate.getY(), poseEstimate.getX(),
                            poseEstimate.getHeading());
                    if (robot.butterfly.positionReached || myTimer.time() > 3000) {
                        reached = true;
                        myTimer.reset();
                        state = State.SCORE;
                    }
                    break;

                case SCORE:
                    switch (score) {
                        case EXTEND:
                            targetPos = (5 - cycles) * coneOffset + webcamOffset;

                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_OUT);
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_OUT);
                            robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                            robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                            robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE);
                            robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);

                            if (reached) {
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                            } else {
                                if (myTimer.time() > 350)
                                    robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);

                            }

                             if (reached) {
                                 if (myTimer.time() > 700) {
                                     myTimer.reset();
                                     reached = false;
                                     score = Score.GRAB;
                                 }
                             }
//                             } else if (myTimer.time() > 1000) {
//                                     myTimer.reset();
//                                     reached = true;
//                                     score = Score.GRAB;
//                             }
                            break;
                        case GRAB:
                            robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                            if (myTimer.time() > 350 && reached) {
                                targetPos += slidesUp;
                                reached = false;
                            }
                            if (myTimer.time() > 650) {
                                robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_INTAKE) / 2);
                                robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                                turret = -180;
                                targetPos = slidesUp;
                            }

                            if (myTimer.time() > 1250) {
                                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                                robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_OUT);
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_OUT);

                                //robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);

                                if (myTimer.time() > 2050) {
                                    robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);
                                    myTimer.reset();
                                    score = Score.UP;
                                }
                            }
                            break;
                        case UP:
//                            targetPos = slidesUp;
//
//                            if (myTimer.time() > 700) {
//                                myTimer.reset();
                                score = Score.EXTENDSCORE;
//                            }
                            break;
                        case EXTENDSCORE:
//                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
//                            robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
//                            robot.intakeOuttake.horizontal.forwardRight.setPosition((FORWARD_RIGHT_OUT + FORWARD_RIGHT_IN) / 2 + .33);
//                            robot.intakeOuttake.horizontal.forwardLeft.setPosition((FORWARD_LEFT_OUT + FORWARD_LEFT_IN) / 2 + .33);

                            if (myTimer.time() > 400) {
                                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE);
                            }

                            if (myTimer.time() > 900) {
                                targetPos -= 500;
                                myTimer.reset();
                                score = Score.REST;
                            }
                            break;
                        case REST:
                            if (myTimer.time() > 250)
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                            if (myTimer.time() > 600) {
                                robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);

                                robot.intakeOuttake.arm.wrist.setPosition((WRIST_INTAKE + WRIST_OUTTAKE) / 2);
                                robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                                robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
                                robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);
                                robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                                turret = 0;
                                targetPos = (5 - cycles) * coneOffset + webcamOffset;
                            }

                            if (myTimer.time() > 850) {
                                //targetPos = webcamOffset;

                                //robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                                //robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);

//                                robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
//                                robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);
//                                robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
//                                robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
                            }

                            if (myTimer.time() > 950) {
                                //robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                                robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_INTAKE) / 2);

                            }

                            if (myTimer.time() > 1150) {
                                if (myTimer.time() > 1500) {
                                    cycles += 1;
                                    myTimer.reset();
                                    score = Score.EXTEND;
                                }
                            }
                            break;
                    }
                    if (cycles == 5)
                        state = State.PARK;
                    break;

                case PARK:

                    robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                    robot.intakeOuttake.arm.arm.setPosition(ARM_REST);
                    robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);

                    targetPos = 0;
                    turret = 0;

                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

                    switch (parkPosition) {
                        case LEFT:
                            robot.butterfly.runToPosition(left.getX(), left.getY(), left.getHeading(),
                                    .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
                                    poseEstimate.getHeading());
                            break;

                        case RIGHT:
                            robot.butterfly.runToPosition(right.getX(), right.getY(), right.getHeading(),
                                    .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
                                    poseEstimate.getHeading());
                            break;

                        case MIDDLE:
                            robot.butterfly.runToPosition(middle.getX(), middle.getY(), middle.getHeading(),
                                    .85, .85, -poseEstimate.getY(), poseEstimate.getX(),
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


            robot.turret.setTurretTargetPosition(turret);

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
        REST
    }
    public Score score;
}
