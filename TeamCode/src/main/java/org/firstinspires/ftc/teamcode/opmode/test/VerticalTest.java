package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.io.Vertical;

@TeleOp(group = "test")
public class VerticalTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Vertical vertical = new Vertical(hardwareMap);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double target = 0;

        waitForStart();

        while (opModeIsActive()) {
            sleep((long) Math.max(0, 10 - timer.time()));
            timer.reset();

            target = Math.max(0, target - 2 * gamepad1.right_stick_y);  // max 200 mm/s
            vertical.setTarget(target);

            telemetry.addData("Vertical Target", "%f mm", vertical.getTarget());
            telemetry.addData("Vertical Position", "%f mm", vertical.getPosition());
            telemetry.update();
            vertical.update();
        }
    }
}
