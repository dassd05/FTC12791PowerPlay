package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.io.Arm;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@TeleOp(group = "test")
public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double target = 0;

        waitForStart();
        
        while (opModeIsActive()) {
            sleep((long) Math.max(0, 10 - timer.time()));
            timer.reset();

            target += .01 * (gamepad1.left_trigger - gamepad1.right_trigger);  // max 60 deg/s = 1.05 rad/s
            arm.setTarget(target);

            arm.stoptake();
            if (gamepad1.a) arm.intake();
            if (gamepad1.b) arm.outtake();

            telemetry.addData("Arm Target", "%f°", Math.toDegrees(arm.getTarget()));
            telemetry.addData("Arm Position", "%f°", Math.toDegrees(arm.getPosition()));
            telemetry.update();
            arm.update();
        }
    }
}
