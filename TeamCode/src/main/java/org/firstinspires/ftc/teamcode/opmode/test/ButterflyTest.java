package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Butterfly;

@TeleOp(group = "test")
public class ButterflyTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Butterfly butterfly = new Butterfly(hardwareMap, null);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) butterfly.setState(Butterfly.State.MECANUM);
            if (gamepad1.y) butterfly.setState(Butterfly.State.TRACTION);
            if (gamepad1.b) butterfly.setState(Butterfly.State.STANDSTILL);

            butterfly.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("State", butterfly.getState().name());
            telemetry.update();
            butterfly.update();
        }
    }
}
