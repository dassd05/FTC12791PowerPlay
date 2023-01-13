package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@TeleOp(group = "test")
public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

//        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        double target = 0;

        waitForStart();

        while (opModeIsActive()) {
//            sleep((long) Math.max(0, 10 - timer.time()));
//            timer.reset();
//
////            target += .02 * (gamepad1.left_trigger - gamepad1.right_trigger);  // max 120 deg/s = 2.09 rad/s
//            target = (gamepad1.left_trigger - gamepad1.right_trigger);  // max 120 deg/s = 2.09 rad/s
//            turret.setTarget(target);

//            telemetry.addData("Turret Target", "%f°", Math.toDegrees(turret.getTarget()));
//            telemetry.addData("Turret Position", "%f°", Math.toDegrees(turret.getPosition()));
//            telemetry.addData("Turret Power", turret.getTarget());
            telemetry.update();
            turret.update(telemetry);
        }
    }
}
