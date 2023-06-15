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

        boolean mecanum = true;

        waitForStart();
        robot.butterfly.setState(Butterfly.State.MECANUM);


        while (opModeIsActive()) {

            if (justPressed.a())
                mecanum = !mecanum;

            if (mecanum)
                robot.butterfly.setState(Butterfly.State.MECANUM);
            else
                robot.butterfly.setState(Butterfly.State.STANDSTILL);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            if (robot.butterfly.getState() == Butterfly.State.MECANUM)
                robot.butterfly.runToPosition(0,0,0,.5,.5, -poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getHeading(), true);
            else
                robot.butterfly.runToPosition(0,0,0,0,.5, -poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getHeading(), true);

            robot.update();
            justPressed.update();

            telemetry.addData("x", -poseEstimate.getY());
            telemetry.addData("y", poseEstimate.getX());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }
}
