package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystem.Constants.DEPLOYMENT.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.io.IntakeOuttake;

@TeleOp(group = "0")
public class RTPTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);
        JustPressed justPressed = new JustPressed(gamepad1);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        robot.butterfly.setState(Butterfly.State.MECANUM);


        while (opModeIsActive()) {
            if (justPressed.x()) robot.butterfly.setState(Butterfly.State.MECANUM);
            if (justPressed.y()) robot.butterfly.setState(Butterfly.State.TRACTION);
            if (justPressed.b() && !gamepad1.start) robot.butterfly.setState(Butterfly.State.STANDSTILL);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            if (gamepad1.a) {
                robot.butterfly.runToPosition(10.0, 15.0, 45,  0.85, 0.85, (-poseEstimate.getY()), poseEstimate.getX(), poseEstimate.getHeading());
            } else {
                double[] fields = { -Math.cbrt(gamepad1.left_stick_y), Math.cbrt(gamepad1.left_stick_x), Math.cbrt(gamepad1.right_stick_x) };
                robot.butterfly.drive(fields[0], fields[1], fields[2]);
            }



            robot.update();
            justPressed.update();

            telemetry.addData("x", -poseEstimate.getY());
            telemetry.addData("y", poseEstimate.getX());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }
}
