package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.io.Turret;

@TeleOp(group = "test")
public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double target = 0;

        waitForStart();

        while (opModeIsActive()) {
            sleep((long) Math.max(0, 10 - timer.time()));
            timer.reset();

            target += .02 * (gamepad1.left_trigger - gamepad1.right_trigger);  // max 120 deg/s = 2.09 rad/s
            turret.setTarget(target);

            telemetry.addData("Turret Target", "%f mm", turret.getTarget());
            telemetry.addData("Turret Position", "%f mm", turret.getPosition());
            telemetry.update();
            turret.update();
        }
    }
}
