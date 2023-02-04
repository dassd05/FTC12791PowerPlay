package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.io.IntakeOuttake;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Disabled
@Config
@TeleOp(group = "test")
public class IntakeOuttakeTest extends LinearOpMode {
    public static double ARM_TARGET = 0;
    public static double TURRET_TARGET = 0;
    public static double HORIZONTAL_TARGET = 0;
    public static double VERTICAL_TARGET = 0;
    public static int INTAKING = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeOuttake intakeOuttake = new IntakeOuttake(hardwareMap, Pose2d::new);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            intakeOuttake.setArmTarget(ARM_TARGET);
            intakeOuttake.setTurretTarget(TURRET_TARGET);
            intakeOuttake.setHorizontalTarget(HORIZONTAL_TARGET);
            intakeOuttake.setVerticalTarget((int) VERTICAL_TARGET);
//            if (INTAKING > 0) intakeOuttake.intake();
//            else if (INTAKING < 0) intakeOuttake.outtake();
//            else intakeOuttake.stoptake();
            telemetry.update();
            intakeOuttake.update();
        }
    }
}
