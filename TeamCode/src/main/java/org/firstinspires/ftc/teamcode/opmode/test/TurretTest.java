package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.subsystem.io.Turret.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.io.Turret;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Config
@TeleOp(group = "test")
public class TurretTest extends LinearOpMode {

    public static int targetPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

//        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        double target = 0;

        turret.setZero(false);

        while (opModeInInit()) {
            turret.update(telemetry);
            telemetry.addData("PID", turret.motor.getPower());
            telemetry.addData("voltage", turret.absoluteEncoder.getVoltage());
            telemetry.addData("zeroed", turret.zeroed());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            turret.setTurretTargetPosition(targetPos);
            turret.update();

//            telemetry.addData("Turret Target", "%f°", Math.toDegrees(turret.getTarget()));
//            telemetry.addData("Turret Position", "%f°", Math.toDegrees(turret.getPosition()));
//            telemetry.addData("Turret Power", turret.getTarget());

            telemetry.addData("turret position", turret.quadratureEncoder.getCurrentPosition());
            telemetry.addData("turret pos", turret.pos);
            telemetry.addData("zeroed", turret.zeroed());
            telemetry.addData("i", totalError * PIDZeroed.i);
            //telemetry.addData("turret error", )
            //telemetry.addData("PID", power1);
            telemetry.update();
        }
    }
}
