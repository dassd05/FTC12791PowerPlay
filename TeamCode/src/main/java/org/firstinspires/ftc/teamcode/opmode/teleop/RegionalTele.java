package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.subsystem.vision.JunctionDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystem.vision.Target;

@TeleOp(name = "RegionalTele", group = "0")
public class RegionalTele extends LinearOpMode {

    public static double P = .0035, I = .0000000000007, D = 25;

    int slidesTargetPos = 0;
    int turretTarget = 0;

    int driverSlides = 0;
    int driverTurret = 0;
    float driverArm = 0;
    double horizontalDriver = 0.0;

    @Override
    public void runOpMode() {

        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime FSMTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Robot robot = new Robot(hardwareMap, telemetry);

        JustPressed justPressed1 = new JustPressed(gamepad1);
        JustPressed justPressed2 = new JustPressed(gamepad2);

        double totalError = 0.0;
        double lastError = 0.0;

        int slidesTop = 1530;
        int slidesMiddle = 870;
        int safe = 20;

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
                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(backLeftIntaking);
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(backRightIntaking);
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
                        linkageAuto= true;
                        FSMTimer.reset();
                        myState = State.SCORE_PREP;
                    }

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

                    if (gamepad1.dpad_down) {
                        driverArm = 0;
                        driverSlides = 0;
                        driverTurret = 0;
                        armNeutral = false;
                        FSMTimer.reset();
                        mySlides = Slides.LOW;
                        myState = State.UP;
                    }
                    if (gamepad1.dpad_up) {
                        driverArm = 0;
                        driverSlides = 0;
                        driverTurret = 0;
                        armNeutral = false;
                        FSMTimer.reset();
                        mySlides = Slides.HIGH;
                        myState = State.UP;
                    }
                    if (gamepad1.dpad_right || gamepad1.dpad_left) {
                        driverArm = 0;
                        driverSlides = 0;
                        driverTurret = 0;
                        armNeutral = false;
                        FSMTimer.reset();
                        mySlides = Slides.MIDDLE;
                        myState = State.UP;
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
                        if (mySlides != Slides.LOW)
                            myState = State.SCORE;
                        else
                            myState = State.DEEXTEND;
                    }
                    break;
                case SCORE:
                    if (mySlides == Slides.LOW) {
                        robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE - .1);
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
                    turretTarget = 0;

                    if (FSMTimer.time() < 350)
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);

                    if (FSMTimer.time() < 750 && FSMTimer.time() > 350) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_SAFE);
                    } else {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                    }

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
            driverTurret += (int) cube(gamepad2.right_stick_x) * (timeDelta / 1e9) * 100;
            driverArm -= cube(gamepad2.left_stick_y) * timeDelta / 1e9 * .15;
            if (myState != State.SCORE_PREP)
                horizontalDriver += timeDelta / 1e9 * .175 * (gamepad1.right_trigger > .5 ? 1.3 : gamepad1.left_trigger > .5 ? -1.3 : 0);
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

            telemetry.update();
            robot.update();
            justPressed1.update();
            justPressed2.update();
        }
    }

    public enum State {
        UP,
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
