package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@TeleOp(name = "TourneyTele", group = "0")
public class TourneyTele extends LinearOpMode {

    public static double P = .0027, I = .0000000000003, D = 35;

    int targetPos = 0;

    @Override
    public void runOpMode() {

        ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Robot robot = new Robot(hardwareMap, telemetry);
        JustPressed justPressed = new JustPressed(gamepad1);

        double totalError = 0.0;
        double lastError = 0.0;

        int slidesTop = 1440;
        int slidesMiddle = 900;
        int safe = 25;

        boolean deployed = false;

        boolean firstTime = true;

        myState = State.DOWN;

        waitForStart();

        myTimer.reset();

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            if (justPressed.x()) robot.butterfly.setState(Butterfly.State.MECANUM);
            if (justPressed.y()) robot.butterfly.setState(Butterfly.State.TRACTION);
            if (justPressed.b() && !gamepad1.start)
                robot.butterfly.setState(Butterfly.State.STANDSTILL);

            // cube root controls means that higher powers (.6-1.0) get spread out to a greater gamepad control (.216-1.00)
            // this allows for more fine movement since our HEAVY robot won't move at lowers powers (< ~.4)
            double[] fields = {-Math.cbrt(gamepad1.left_stick_y), Math.cbrt(gamepad1.left_stick_x), Math.cbrt(gamepad1.right_stick_x)};
            // slowdown
            if (gamepad1.right_bumper)
                fields = new double[]{fields[0] * .35, fields[1] * .35, fields[2] * .5};
            else robot.butterfly.drive(fields[0], fields[1], fields[2]);


            if (justPressed.dpad_up()) {
                firstTime = false;
                myState = State.HIGH;
                deployed = false;
                myTimer.reset();
            }
            if (justPressed.dpad_down()) {
                firstTime = false;
                myState = State.DOWN;
                deployed = false;
                myTimer.reset();
            }
            if (justPressed.dpad_right()) {
                firstTime = false;
                myState = State.MIDDLE;
                deployed = false;
                myTimer.reset();
            }
            if (justPressed.dpad_left()) {
                firstTime = false;
                myState = State.LOW;
                deployed = false;
                myTimer.reset();
            }

            switch (myState) {
                case DOWN:
                    if (firstTime) {
                        targetPos = safe;

                        robot.intakeOuttake.arm.arm.setPosition(ARM_INTAKE);
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
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
                    }
                    //firstTime = false;

                    robot.intakeOuttake.horizontal.backwardLeft.setPosition((BACKWARD_LEFT_IN + BACKWARD_LEFT_OUT) / 2);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition((BACKWARD_RIGHT_IN + BACKWARD_RIGHT_OUT) / 2);

                    if (justPressed.a()) {
                        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
                        myTimer.reset();
                        myState = State.INTAKE;
                    }
                    break;
                case INTAKE:
                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);

                    if (myTimer.time() > 175)
                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(500, myTimer.time(), 675, ARM_INTAKE, (ARM_REST + ARM_OUTTAKE) / 2));

                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);

                    if (myTimer.time() > 500) {
                        robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);
                    }

                    break;
                case MIDDLE:
                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    if (deployed)
                        targetPos -= 500;
                    else
                        targetPos = slidesMiddle;
                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(500, myTimer.time(), 500, (ARM_REST + ARM_OUTTAKE) / 2, ARM_OUTTAKE));

                    if (justPressed.left_bumper()) {
                        deployed = true;
                    }

                    break;
                case HIGH:
                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    if (deployed)
                        targetPos -= 500;
                    else
                        targetPos = slidesTop;
                    robot.intakeOuttake.arm.arm.setPosition(linearProfile(800, myTimer.time(), 800, (ARM_REST + ARM_OUTTAKE) / 2, ARM_OUTTAKE));

                    if (justPressed.left_bumper()) {
                        deployed = true;
                    }
                    break;
                case LOW:
                    robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                    robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                    if (deployed)
                        robot.intakeOuttake.arm.arm.setPosition(linearProfile(300, myTimer.time(), 300, (ARM_REST + ARM_OUTTAKE) / 2, ARM_OUTTAKE));
                    else
                        robot.intakeOuttake.arm.arm.setPosition((ARM_REST + ARM_OUTTAKE) / 2);

                    if (justPressed.left_bumper()) {
                        deployed = true;
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


            robot.turret.setTurretTargetPosition(0);
            robot.update();
            justPressed.update();

        }
    }

    public enum State {
        LOW,
        MIDDLE,
        HIGH,
        DOWN,
        INTAKE
    }
    public State myState;

    public double linearProfile(double totalTime, double currentTime, double endTime, double initial, double fin) {
        return (initial + (fin - initial) * (1 - (endTime - currentTime) / totalTime));
    }
}
