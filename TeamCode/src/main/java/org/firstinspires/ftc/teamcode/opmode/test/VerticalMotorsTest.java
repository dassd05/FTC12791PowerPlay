package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.MultiMotor;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Config
@TeleOp(group = "test")
public class VerticalMotorsTest extends LinearOpMode {
    public static int TARGET_POSITION = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MultiMotor vertical = new MultiMotor(hardwareMap, "vertical1", "vertical2", "vertical3");
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int lastError = 0;
        int totalError = 0;
        long lastTime = System.nanoTime();;

        telemetry = TelemetryUtil.initTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            long time = System.nanoTime();
            int position = vertical.getCurrentPosition();
            int error = TARGET_POSITION - position;
            if (error < 0 != lastError < 0) totalError = 0;
            else totalError += (error - lastError) * (time - lastTime);
            double d = (double) (error - lastError) / (time - lastTime);

            double power = kP * error + (Math.abs(totalError * kI) < .3 ? totalError * kI : .3) + kD * d;
            vertical.setPower(power);

            double[] currents = vertical.getCurrents(CurrentUnit.AMPS);
            telemetry.addData("power", power);
            telemetry.addData("error", error);
            telemetry.addData("total error", totalError);
            telemetry.addData("delta error", d);
            telemetry.addData("position", vertical.getCurrentPosition());
            telemetry.addData("velocity", vertical.getVelocity());
            for (int i = 0; i < currents.length; i++) {
                telemetry.addData("current" + i + " (amps)", currents[i]);
            }
            telemetry.update();
            lastError = error;
            lastTime = time;
        }
    }
}
