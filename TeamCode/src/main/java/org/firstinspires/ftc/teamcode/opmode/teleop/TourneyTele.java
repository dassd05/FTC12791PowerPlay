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

@TeleOp(name = "TourneyTele", group = "0")
public class TourneyTele extends LinearOpMode {

//    public static double P = .0027, I = .0000000000003, D = 35;
    public static double P = .0035, I = .0000000000007, D = 25;

    int targetPos = 0;

    @Override
    public void runOpMode() {

        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Robot robot = new Robot(hardwareMap, telemetry);
//        JunctionDetectionPipeline junctionDetectionPipeline = new JunctionDetectionPipeline();
//        Webcam webcam = new Webcam(hardwareMap, "Webcam 2", junctionDetectionPipeline);
        //FtcDashboard.getInstance().startCameraStream(webcam.cvCamera, 60);
        JustPressed justPressed1 = new JustPressed(gamepad1);
        JustPressed justPressed2 = new JustPressed(gamepad2);

        double totalError = 0.0;
        double lastError = 0.0;

        int slidesTop = 1450;
        int slidesMiddle = 900;
        int safe = 30;

        double backLeftIntaking = Range.scale(.75, 0, 1, BACKWARD_LEFT_IN, BACKWARD_LEFT_OUT);
        double backRightIntaking = Range.scale(.75, 0, 1, BACKWARD_RIGHT_IN, BACKWARD_RIGHT_OUT);

        boolean deployed = false;

        boolean firstTime = true;

        myState = State.DOWN;

        boolean armNeutral = true;

        Target junction = null, lastJunction = null;
        boolean junctionDetectionEnabled = true;
        int turretTarget = 0;
        double horizontalTarget = 0;
        State verticalTarget = null;
        int driverTurretAdjust = 0;
        double driverArmAdjust = 0;
        int driverVerticalAdjust = 0;
        double driverHorizontalAdjust = 0;
        boolean deployShortcut = false;
        boolean deployDownQueued = false;


        robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
        robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);


        waitForStart();

        myTimer.reset();

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            if (justPressed1.x()) robot.butterfly.setState(Butterfly.State.MECANUM);
//            if (justPressed1.y()) robot.butterfly.setState(Butterfly.State.TRACTION);
            if (justPressed1.b() && !gamepad1.start)
                robot.butterfly.setState(Butterfly.State.STANDSTILL);

            // cube root controls means that higher powers (.6-1.0) get spread out to a greater gamepad control (.216-1.00)
            // this allows for more fine movement since our HEAVY robot won't move at lowers powers (< ~.4)
            double[] fields = {-Math.cbrt(gamepad1.left_stick_y), Math.cbrt(gamepad1.left_stick_x), Math.cbrt(gamepad1.right_stick_x)};
            // slowdown
            if (gamepad1.right_bumper)
                fields = new double[]{fields[0] * .35, fields[1] * .35, fields[2] * .5};
            robot.butterfly.drive(fields[0], fields[1], fields[2]);

            if (justPressed1.left_bumper()) {
                deployed = true;
            }

            if (justPressed2.left_bumper() && gamepad2.dpad_up || gamepad2.left_bumper && justPressed2.dpad_up()) {
                deployShortcut = true;
                horizontalTarget = 1;
                verticalTarget = State.HIGH;
                turretTarget = Turret.radiansToTicks(.55);
            } else if (justPressed2.left_bumper() && (gamepad2.dpad_left || gamepad2.dpad_right) || gamepad2.left_bumper && (justPressed2.dpad_left() || justPressed2.dpad_right())) {
                deployShortcut = true;
                horizontalTarget = .4;
                verticalTarget = State.MIDDLE;
                turretTarget = Turret.radiansToTicks(1.1);
            } else if (justPressed2.left_bumper() && gamepad2.dpad_down || gamepad2.left_bumper && justPressed2.dpad_down()) {
                deployShortcut = true;
                horizontalTarget = -.4;
                verticalTarget = State.LOW;
                turretTarget = Turret.radiansToTicks(-1.1);
            } else if (justPressed2.right_bumper() && gamepad2.dpad_up || gamepad2.right_bumper && justPressed2.dpad_up()) {
                deployShortcut = true;
                horizontalTarget = 1;
                verticalTarget = State.HIGH;
                turretTarget = Turret.radiansToTicks(-.55);
            } else if (justPressed2.right_bumper() && (gamepad2.dpad_left || gamepad2.dpad_right) || gamepad2.right_bumper && (justPressed2.dpad_left() || justPressed2.dpad_right())) {
                deployShortcut = true;
                horizontalTarget = .4;
                verticalTarget = State.MIDDLE;
                turretTarget = Turret.radiansToTicks(-1.1);
            } else if (justPressed2.right_bumper() && gamepad2.dpad_down || gamepad2.right_bumper && justPressed2.dpad_down()) {
                deployShortcut = true;
                horizontalTarget = -.4;
                verticalTarget = State.LOW;
                turretTarget = Turret.radiansToTicks(1.1);
            } else if (justPressed2.dpad_up()) {
                deployShortcut = true;
                horizontalTarget = 0;
                verticalTarget = State.HIGH;
                turretTarget = 0;
            } else if (justPressed2.dpad_left() || justPressed2.dpad_right()) {
                deployShortcut = true;
                horizontalTarget = 0;
                verticalTarget = State.MIDDLE;
                turretTarget = 0;
            } /*else if (justPressed2.dpad_down()) {
                deployShortcut = true;
                horizontalTarget = 0;
                verticalTarget = State.LOW;
                turretTarget = 0;
            }*/ else if (justPressed2.dpad_down()) {
                deployDownQueued = true;
            }

            if (deployShortcut) {
                deployShortcut = false;
                firstTime = false;
                myState = State.DEPLOYING;
                deployed = false;
                //junctionDetectionEnabled = true;
                driverTurretAdjust = 0;
                driverArmAdjust = 0;
                driverHorizontalAdjust = 0;
                driverVerticalAdjust = 0;
                myTimer.reset();
            }

            long timeDelta = System.nanoTime() - lastTime;
            driverTurretAdjust += (int) cube(gamepad2.right_stick_x) * (timeDelta / 1e9) * 100;
            driverArmAdjust -= cube(gamepad2.left_stick_y) * timeDelta / 1e9 * .15;
            horizontalTarget += timeDelta / 1e9 * .175 * (gamepad1.dpad_right ? 1 : gamepad1.dpad_left ? -1 : 0);
            driverVerticalAdjust += timeDelta / 1e9 * 300 * (gamepad1.dpad_up ? 1 : gamepad1.dpad_down ? -1 : 0);
            //junctionDetectionEnabled = driverTurretAdjust != 0;

//            if (gamepad2.x) {
//                junctionDetectionEnabled = false;
//                webcam.cvCamera.stopStreaming();
//                junction = lastJunction = null;
//                junctionDetectionPipeline = null;
//            }

            if (justPressed1.y()) armNeutral = !armNeutral;

            switch (myState) {
                case DOWN:
                    if (firstTime) {
                        targetPos = safe;

                        robot.intakeOuttake.arm.arm.setPosition(armNeutral ? .693 : ARM_INTAKE);
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);

                        turretTarget = 0;
                    }
                    if (!firstTime) {
                        targetPos = safe;

                        if (myTimer.time() < 350)
                            robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);

                        if (myTimer.time() < 750 && myTimer.time() > 350) {
                            robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                            robot.intakeOuttake.arm.wrist.setPosition(WRIST_SAFE);
                        } else {
                            robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                            robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
                        }

                        if (myTimer.time() < 750)
                            robot.intakeOuttake.arm.arm.setPosition(linearProfile(750, myTimer.time(), 750, ARM_OUTTAKE, ARM_INTAKE));
                        else
                            robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE);

                        robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);

                        if (robot.intakeOuttake.horizontal.forwardRight.getPosition() != FORWARD_RIGHT_IN) {
                            robot.intakeOuttake.horizontal.forwardRight.setPosition(linearProfile(750, myTimer.time(), 750, FORWARD_RIGHT_OUT, FORWARD_RIGHT_IN));
                            robot.intakeOuttake.horizontal.forwardLeft.setPosition(linearProfile(750, myTimer.time(), 750, FORWARD_LEFT_OUT, FORWARD_LEFT_IN));
                        } else {
                            robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                            robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
                        }

                        if (myTimer.time() > 750) {
                            turretTarget = 0;
                        }
                    }
                    //firstTime = false;

                    if (justPressed1.a()) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                        myTimer.reset();
                        myState = State.INTAKING;
                    }
                    break;
                case INTAKING:
                    //driverVerticalAdjust = 25;
                    if (myTimer.time() < 700) {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(700, myTimer.time(), 700, (BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2, backLeftIntaking));
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(700, myTimer.time(), 700, (BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2, backRightIntaking));
                    } else {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(backLeftIntaking);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(backRightIntaking);
                    }
                    robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE);
                    if (myTimer.time() > 400)
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);

                    if (justPressed1.a()) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                        //sleep(300);
                        myTimer.reset();
                        myState = State.INTAKE;
                    }
                    break;
                case INTAKE:
                    if (myTimer.time() < 1200 && myTimer.time() > 500) {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(700, myTimer.time(), 1200, backLeftIntaking, BACKWARD_LEFT_IN));
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(700, myTimer.time(), 1200, backRightIntaking, BACKWARD_RIGHT_IN));
                    }
                    robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
//                    if (gamepad1.)

                    if (myTimer.time() > 675 && myTimer.time() < 1175)
                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(500, myTimer.time(), 1175, ARM_INTAKE, (ARM_REST)));
                    if (myTimer.time() > 1175)
                        robot.intakeOuttake.arm.arm.setPosition(ARM_REST);

                    if (myTimer.time() > 1000) {
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                    }

                    if (justPressed1.a()) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                        myTimer.reset();
                        myState = State.DOWN;
                    }
                    break;
//                case MIDDLE:
//                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
//                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
//                    if (deployed)
//                        targetPos = slidesMiddle - 500;
//                    else
//                        targetPos = slidesMiddle;
//
//                    if (myTimer.time() < 500)
//                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(500, myTimer.time(), 500, ARM_REST, ARM_OUTTAKE));
//
//                    if (justPressed1.left_bumper()) {
//                        deployed = true;
//                    }
//
//                    break;
//                case HIGH:
//                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
//                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
//                    robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
//                    robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
//                    if (deployed)
//                        targetPos = slidesTop - 500;
//                    else
//                        targetPos = slidesTop;
//
//                    if (myTimer.time() < 800)
//                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(800, myTimer.time(), 800, ARM_REST , ARM_OUTTAKE));
//
//                    if (justPressed1.left_bumper()) {
//                        deployed = true;
//                    }
//                    break;
//                case LOW:
//                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
//                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
//                    if (deployed)
//                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(300, myTimer.time(), 300, (ARM_REST + ARM_OUTTAKE) / 2, ARM_OUTTAKE));
//                    else
//                        robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);
//
//                    if (justPressed1.left_bumper()) {
//                        deployed = true;
//                    }
//                    break;
                case DEPLOYING:
                    targetPos = verticalTarget == State.LOW ? 300 : verticalTarget == State.MIDDLE ? slidesMiddle : verticalTarget == State.HIGH ? slidesTop : 0;
                    if (deployed) targetPos -= 500;

                    if (horizontalTarget > 0) {
                        if (myTimer.time() < (700 * horizontalTarget) + 700 && myTimer.time() > 700) {
                            robot.intakeOuttake.horizontal.forwardLeft.setPosition(linearProfile(700 * horizontalTarget, myTimer.time() - 700, 700 * horizontalTarget, FORWARD_LEFT_IN, Range.scale(horizontalTarget, 0, 1, FORWARD_LEFT_IN, FORWARD_LEFT_OUT)));
                            robot.intakeOuttake.horizontal.forwardRight.setPosition(linearProfile(700 * horizontalTarget, myTimer.time() - 700, 700 * horizontalTarget, FORWARD_RIGHT_IN, Range.scale(horizontalTarget, 0, 1, FORWARD_RIGHT_IN, FORWARD_RIGHT_OUT)));
                        } else {
                            robot.intakeOuttake.horizontal.forwardLeft.setPosition(Range.scale(horizontalTarget, 0, 1, FORWARD_LEFT_IN, FORWARD_LEFT_OUT));
                            robot.intakeOuttake.horizontal.forwardRight.setPosition(Range.scale(horizontalTarget, 0, 1, FORWARD_RIGHT_IN, FORWARD_RIGHT_OUT));
                        }
                    } else if (horizontalTarget < 0) {
                        if (myTimer.time() < (-700 * horizontalTarget) + 700 && myTimer.time() > 700) {
                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(linearProfile(-700 * horizontalTarget, myTimer.time() - 700, -700 * horizontalTarget, BACKWARD_LEFT_IN, Range.scale(horizontalTarget, 0, 1, BACKWARD_LEFT_IN, BACKWARD_LEFT_OUT)));
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(linearProfile(-700 * horizontalTarget, myTimer.time() - 700, -700 * horizontalTarget, BACKWARD_RIGHT_IN, Range.scale(horizontalTarget, 0, 1, BACKWARD_RIGHT_IN, BACKWARD_RIGHT_OUT)));
                        } else {
                            robot.intakeOuttake.horizontal.backwardLeft.setPosition(Range.scale(horizontalTarget, 0, 1, BACKWARD_LEFT_IN, BACKWARD_LEFT_OUT));
                            robot.intakeOuttake.horizontal.backwardRight.setPosition(Range.scale(horizontalTarget, 0, 1, BACKWARD_RIGHT_IN, BACKWARD_RIGHT_OUT));
                        }
                    } else {
                        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                        robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
                        robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                    }

                    if (myTimer.time() < 1500 && myTimer.time() > 700)
                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(800, myTimer.time(), 1500, ARM_REST , ARM_OUTTAKE));
                    else if (myTimer.time() > 1500)
                        robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE + driverArmAdjust);

//                    if (junction != lastJunction && junction != null && Math.abs(robot.turret.quadratureEncoder.getRawVelocity()) < 5 && junctionDetectionEnabled) {
//                        turretTarget = -robot.turret.quadratureEncoder.getCurrentPosition() + Turret.radiansToTicks(Math.toRadians(junction.offset));
//                    }

                    if (deployDownQueued) {
                        deployDownQueued = false;
                        myState = State.DOWN;
                        deployed = false;
                        //junctionDetectionEnabled = true;
//                        turretTarget = 0;
                        driverTurretAdjust = 0;
                        driverArmAdjust = 0;
                        driverHorizontalAdjust = 0;
                        driverVerticalAdjust = 0;
                        myTimer.reset();
                    }
                    break;
            }


            long time = System.nanoTime();
            double error = Math.max(safe, targetPos + driverVerticalAdjust) - robot.vertical.v2.getCurrentPosition();
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

            robot.turret.setTurretTargetPosition(turretTarget + driverTurretAdjust);

//            if (junctionDetectionEnabled && junctionDetectionPipeline != null) {
//                lastJunction = junction;
//                junction = junctionDetectionPipeline.getMostCenteredJunction();
//            }

//            robot.turret.setTurretTargetPosition(0);

            telemetry.addData("targetPosition", targetPos + driverVerticalAdjust);
            telemetry.addData("turret adjustment", driverTurretAdjust);
            telemetry.addData("gamepad2.right_stick_x", gamepad2.right_stick_x);
            telemetry.addData("theoretical adjustment", cube(gamepad2.right_stick_x) * (timeDelta / 1e9) * 30);
            telemetry.update();
            robot.update();
            justPressed1.update();
            justPressed2.update();
        }
    }

    public enum State {
        LOW,
        MIDDLE,
        HIGH,
        DOWN,
        INTAKE,
        INTAKING,
        DEPLOYING,
    }
    public State myState;

    public double linearProfile(double totalTime, double currentTime, double endTime, double initial, double fin) {
        return (initial + (fin - initial) * (1 - (endTime - currentTime) / totalTime));
    }

    public static double cube(double x) {
        return x*x*x;
    }
}
