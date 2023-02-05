package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;
import static org.firstinspires.ftc.teamcode.subsystem.io.Arm.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.io.IntakeOuttake;

@TeleOp(group = "demo")
public class JudgingShowcase extends LinearOpMode {

    public static double P = .0027, I = .0000000000003, D = 35;

    int targetPos = 0;
    boolean mecanum = true;
    boolean turret = false;

    boolean arm = false;
    boolean wrist = false;
    boolean claw = false;

    public ElapsedTime ioTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);
        JustPressed justPressed = new JustPressed(gamepad1);
        boolean deploying = false;

        double totalError = 0.0;
        double lastError = 0.0;

        robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
        robot.intakeOuttake.arm.arm.setPosition(.693);
        robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);

        robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
        robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
        robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
        robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);

        waitForStart();

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            if (justPressed.a())
                mecanum = !mecanum;

            if (mecanum)
                robot.butterfly.setState(Butterfly.State.MECANUM);
            else
                robot.butterfly.setState(Butterfly.State.TRACTION);

            if (justPressed.x())
                turret = !turret;

            if (turret)
                robot.turret.setTurretTargetPosition(500);
            else
                robot.turret.setTurretTargetPosition(0);

            if (gamepad1.dpad_left) {
                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_OUT);
                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_OUT);
                robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
            }
            if (gamepad1.dpad_right) {
                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_OUT);
                robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_OUT);
            }

            if (justPressed.b())
                arm = !arm;
            if (justPressed.y())
                claw = !claw;
            if (justPressed.left_bumper())
                wrist = !wrist;


            if (arm)
                robot.intakeOuttake.arm.arm.setPosition(.693);
            else
                robot.intakeOuttake.arm.arm.setPosition(ARM_OUTTAKE);

            if (claw)
                robot.intakeOuttake.arm.claw.setPosition(CLAW_CLOSE);
            else
                robot.intakeOuttake.arm.claw.setPosition(CLAW_OPEN);

            if (wrist)
                robot.intakeOuttake.arm.wrist.setPosition(WRIST_INTAKE);
            else
                robot.intakeOuttake.arm.wrist.setPosition(WRIST_OUTTAKE);

            if (gamepad1.dpad_up)
                targetPos = 300;
            if (gamepad1.dpad_down) {
                targetPos = 0;
                claw = true;
                arm = true;
                wrist = true;

                robot.intakeOuttake.horizontal.backwardLeft.setPosition(BACKWARD_LEFT_IN);
                robot.intakeOuttake.horizontal.backwardRight.setPosition(BACKWARD_RIGHT_IN);
                robot.intakeOuttake.horizontal.forwardRight.setPosition(FORWARD_RIGHT_IN);
                robot.intakeOuttake.horizontal.forwardLeft.setPosition(FORWARD_LEFT_IN);
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

            robot.update();
            justPressed.update();
        }
    }
}
