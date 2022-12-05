package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.io.Arm;

public class ArmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);

        JustPressed justPressed = new JustPressed(gamepad1);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double target = 0;
        boolean intaked = false;
        
        waitForStart();
        
        while (opModeIsActive()) {
            sleep((long) Math.max(0, 10 - timer.time()));
            timer.reset();

            target += .01 * (gamepad1.left_trigger - gamepad1.right_trigger);  // max 60 deg/s = 1.05 rad/s
            arm.setTarget(target);

            if (justPressed.a()) {
                intaked = !intaked;
                if (intaked) arm.intake();
                else arm.outtake();
            }

            telemetry.addData("Arm Target", "%f mm", arm.getTarget());
            telemetry.addData("Arm Position", "%f mm", arm.getPosition());
            telemetry.addData("Intaked", intaked);
            telemetry.update();
            arm.update();
        }
    }
}
