package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Odometry;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Disabled
@TeleOp(group = "test")
public class OdometryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new Odometry(hardwareMap);
        String[] wheelNames = new String[]{ "left", "right", "front" };
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < 3; i++) telemetry.addData(wheelNames[i] + " position", odometry.getWheelPositions().get(i));
            for (int i = 0; i < 3; i++) telemetry.addData(wheelNames[i] + "velocity", odometry.getWheelVelocities().get(i));
            telemetry.addData("position", odometry.getPoseEstimate().toString());
            if (odometry.getPoseVelocity() != null) telemetry.addData("velocity", odometry.getPoseVelocity().toString());
            telemetry.update();
            odometry.update();
        }
    }
}
