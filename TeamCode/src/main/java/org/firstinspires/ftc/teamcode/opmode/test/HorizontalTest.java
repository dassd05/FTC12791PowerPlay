package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.io.Horizontal;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Disabled
@TeleOp(group = "test")
public class HorizontalTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Horizontal horizontal = new Horizontal(hardwareMap);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double target = 0;

        waitForStart();

        while (opModeIsActive()) {
            sleep((long) Math.max(0, 10 - timer.time()));
            timer.reset();

            target += 2 * gamepad1.right_stick_x;  // max 200 mm/s
            horizontal.setTarget(target);

            telemetry.addData("Horizontal Target", "%f mm", horizontal.getTarget());
            telemetry.addData("Horizontal Position", "%f mm", horizontal.getPosition());
            telemetry.update();
            horizontal.update();
        }
    }
}
