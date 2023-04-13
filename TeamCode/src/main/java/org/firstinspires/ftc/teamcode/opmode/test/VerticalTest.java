package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.io.Vertical;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Config
@TeleOp(group = "test")
public class VerticalTest extends LinearOpMode {

    public static double target = 0;
    public static double P = .007, I = 7e-11 , D = 400;

    double totalError = 0.0;
    double lastError = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Vertical vertical = new Vertical(hardwareMap);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        waitForStart();

        long lastTime = System.nanoTime();

        while (opModeIsActive()) {

            long time = System.nanoTime();
            double error = target - vertical.v2.getCurrentPosition();
            if (error < 0 != lastError < 0) totalError = 0;
            else totalError += (error) * (time - lastTime);
            double d = (error - lastError) / (time - lastTime);

            double pError = P * error;
            double i = totalError * I;
            double iError = (Math.abs(i) < .4 ? i : Math.signum(i) * .4);
            double dError = D * d;

            double power = (pError + iError + dError);
            lastTime = time;
            lastError = error;

            vertical.v1.setPower(power);
            vertical.v2.setPower(power);
            vertical.v3.setPower(power);
            telemetry.update();
        }
    }
}
