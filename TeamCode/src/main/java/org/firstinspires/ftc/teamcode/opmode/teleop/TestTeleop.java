package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.io.IntakeOuttake;

@TeleOp(group = "0")
public class TestTeleop extends LinearOpMode {

    boolean slidesReached = false;
    boolean intake = false;

    double outtakePos = 0.0;
    double intakePos = 0.0;

    double intakeSpeed = 0.0;

    public static double P = .0027, I = .0000000000003, D = 35;

    public ElapsedTime ioTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);
        JustPressed justPressed = new JustPressed(gamepad1);
        boolean deploying = false;

        double totalError = 0.0;
        double lastError = 0.0;

        io = IO.REST;
        IOUpdate();

        waitForStart();

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            if (justPressed.x()) robot.butterfly.setState(Butterfly.State.MECANUM);
            if (justPressed.y()) robot.butterfly.setState(Butterfly.State.TRACTION);
            if (justPressed.b() && !gamepad1.start) robot.butterfly.setState(Butterfly.State.STANDSTILL);

            if (!deploying) {
                // cube root controls means that higher powers (.6-1.0) get spread out to a greater gamepad control (.216-1.00)
                // this allows for more fine movement since our HEAVY robot won't move at lowers powers (< ~.4)
                double[] fields = { -Math.cbrt(gamepad1.left_stick_y), Math.cbrt(gamepad1.left_stick_x), Math.cbrt(gamepad1.right_stick_x) };
                // slowdown
                if (gamepad1.right_bumper) fields = new double[] { fields[0] * .35, fields[1] * .35, fields[2] * .5 };
                else robot.butterfly.drive(fields[0], fields[1], fields[2]);
            }
            else robot.butterfly.brake();


            if (justPressed.a() && !gamepad1.start) deploying = !deploying;


            if (io == IO.OUTTAKE) {
                if (Math.abs(robot.vertical.v2.getCurrentPosition() - targetPos) < 25 || ioTimer.time() > 2000)
                    slidesReached = true;
                else
                    slidesReached = false;
            }
            //robot.vertical.setTarget(targetPos);

            robot.intakeOuttake.horizontal.backwardLeft.setPosition(intakeLinkage1In + intakePos);
            robot.intakeOuttake.horizontal.backwardRight.setPosition(intakeLinkage2In - intakePos);
            robot.intakeOuttake.horizontal.forwardRight.setPosition(outtakeLinkage1In + outtakePos );
            robot.intakeOuttake.horizontal.forwardLeft.setPosition(outtakeLinkage2In - outtakePos);


            if (gamepad1.left_bumper)
                intakeSpeed = -1.0;

            if (gamepad1.a)
                intake = true;

            if (gamepad1.dpad_down)
                intake();

            if (gamepad1.dpad_up)
                outtake();

            if (gamepad1.dpad_left || gamepad1.dpad_right)
                rest();

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

            IOUpdate();

            robot.intakeOuttake.arm.arm.setPosition(arm);
            robot.intakeOuttake.arm.intake.setPower(intakeSpeed);

            robot.vertical.v1.setPower(power + .01);
            robot.vertical.v2.setPower(power + .01);
            robot.vertical.v3.setPower(power +.01);

            //robot.vertical.update(targetPos);

            telemetry.addData("slides power 1", robot.vertical.v2.getPower());
            telemetry.addData("slides power 2", robot.vertical.v1.getPower());
            telemetry.addData("slides power 3", robot.vertical.v3.getPower());
            telemetry.addData("slides encoder", robot.vertical.v2.getCurrentPosition());
            telemetry.addData("arm pos", arm);
            telemetry.addData("p", pError);
            telemetry.addData("i", iError);
            telemetry.addData("d", dError);
            robot.update();
            justPressed.update();
        }
    }

    public void setIntakeLinkage(double target) {
        intakePos = target;
    }

    public void setOuttakeLinkage(double target) {
        outtakePos = target;
    }
    public enum IO {
        INTAKE,
        OUTTAKE,
        REST
    }

    public IO io;

    public int targetPos = 0;

    public double arm = 0;
    public void IOUpdate() {
        switch(io) {
            case OUTTAKE:
                setIntakeLinkage(0);
                setOuttakeLinkage(.31);
                intake = false;

                if (ioTimer.time() < 750)
                    intakeSpeed = 1.0;

                targetPos = slidesUp;

                if (slidesReached)
                    arm = v4bOuttake;
                else
                    arm = v4bNeutral;
                break;
            case INTAKE:
                targetPos = 0;
                setIntakeLinkage(.4);
                setOuttakeLinkage(.16);
                if (intake) {
                    arm = v4bIntake;
                    intakeSpeed = 1.0;
                }
                else
                    arm = v4bIntakePrep;
                break;
            case REST:
                intake = false;
                targetPos = 0;
                arm = v4bNeutral;
                setIntakeLinkage(0);
                setOuttakeLinkage(.16);

                if (ioTimer.time() < 500)
                    intakeSpeed = 1.0;
                else
                    intakeSpeed = 0.0;
                break;

        }
    }

    public void rest() {
        ioTimer.reset();
        io = IO.REST;
    }
    public void intake()  {
        ioTimer.reset();
        io = IO.INTAKE;
    }

    public void outtake() {
        ioTimer.reset();
        io = IO.OUTTAKE;
    }
}
