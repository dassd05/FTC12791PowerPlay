package org.firstinspires.ftc.teamcode.opmode.test;

import static org.firstinspires.ftc.teamcode.opmode.test.HardwareTest.servoCurrent;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.Butterfly;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Disabled
@TeleOp(group = "test")
public class ButterflyTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Butterfly butterfly = new Butterfly(hardwareMap, Pose2d::new);
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) butterfly.setState(Butterfly.State.MECANUM);
            if (gamepad1.y) butterfly.setState(Butterfly.State.STANDSTILL);
            if (gamepad1.b) butterfly.setState(Butterfly.State.BAD_TRACTION);
            if (gamepad1.a) butterfly.setState(Butterfly.State.TRACTION);

            double[] fields = {-Math.cbrt(gamepad1.left_stick_y), Math.cbrt(gamepad1.left_stick_x), Math.cbrt(gamepad1.right_stick_x)};
            if (gamepad1.right_bumper)
                fields = new double[]{fields[0] * .35, fields[1] * .35, fields[2] * .5};
            butterfly.drive(fields[0], fields[1], fields[2]);

            telemetry.addData("State", butterfly.getState().name());
            for (DcMotorEx motor :
                    new DcMotorEx[]{butterfly.backLeft, butterfly.backRight,
                    butterfly.frontLeft, butterfly.frontRight})
                telemetry.addData("Motor" + motor.getDeviceName() + " Current (amps)",
                        motor.getCurrent(CurrentUnit.AMPS));
            for (LynxModule lynxModule : hardwareMap.getAll(LynxModule.class))
                telemetry.addData(String.format("<code>%s</code> \"%s\" Servo Current (amps)",
                        lynxModule.getDeviceName(), hardwareMap.getNamesOf(lynxModule).iterator().next()), servoCurrent(lynxModule));
            telemetry.update();
            butterfly.update();
        }
    }
}
