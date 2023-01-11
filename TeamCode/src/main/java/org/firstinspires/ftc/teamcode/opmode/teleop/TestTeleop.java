package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.io.IntakeOuttake;

@TeleOp(group = "test")
public class TestTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);
        JustPressed justPressed = new JustPressed(gamepad1);
        boolean deploying = false;

        waitForStart();

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
                // field centric
                if (gamepad1.left_bumper) robot.butterfly.driveFieldCentric(fields[0], fields[1], fields[2]);
                else robot.butterfly.drive(fields[0], fields[1], fields[2]);
            }
            else robot.butterfly.brake();

            if (justPressed.dpad_down()) {
                deploying = false;
                robot.intakeOuttake.deploy(IntakeOuttake.JunctionLevel.NONE);
            }
            if (justPressed.dpad_left()) {
                deploying = true;
                robot.intakeOuttake.deploy(IntakeOuttake.JunctionLevel.LOW);
            }
            if (justPressed.dpad_right()) {
                deploying = true;
                robot.intakeOuttake.deploy(IntakeOuttake.JunctionLevel.MID);
            }
            if (justPressed.dpad_up()) {
                deploying = true;
                robot.intakeOuttake.deploy(IntakeOuttake.JunctionLevel.HIGH);
            }

            if (deploying) {
                robot.intakeOuttake.adjustVerticalNoSlides(-gamepad1.left_stick_y);
                robot.intakeOuttake.adjustHorizontalTarget(gamepad1.left_stick_x);
//                robot.intakeOuttake.adjustTurretTarget(-.01 * gamepad1.right_stick_x);
                robot.intakeOuttake.adjustArmStayStill(.01 * gamepad1.right_stick_y);
                if (gamepad1.a && !gamepad1.start) robot.intakeOuttake.outtake();
            } else {
                if (gamepad1.a && !gamepad1.start) robot.intakeOuttake.intake();
            }
            if (!gamepad1.a) robot.intakeOuttake.stoptake();

            robot.intakeOuttake.adjustTurretTarget(.01 * (gamepad1.left_trigger - gamepad1.right_trigger));

            telemetry.addData("Drivetrain State", robot.butterfly.getState().name());
            telemetry.addData("Position", robot.getPosition().toString());
            telemetry.addData("Orientation", robot.getOrientation().toAngleUnit(AngleUnit.DEGREES).toString());
            telemetry.addData("Update Rate", robot.getUpdateRate());
            robot.update();
            justPressed.update();
        }
    }
}
