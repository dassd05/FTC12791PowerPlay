package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

@Config
@TeleOp(group = "test")
public class HardwareTest extends LinearOpMode {
    public static double MOTOR0_POWER = 0;
    public static double MOTOR1_POWER = 0;
    public static double MOTOR2_POWER = 0;
    public static double MOTOR3_POWER = 0;
    public static double SERVO0_POSITION = -1;
    public static double SERVO1_POSITION = -1;
    public static double SERVO2_POSITION = -1;
    public static double SERVO3_POSITION = -1;
    public static double SERVO4_POSITION = -1;
    public static double SERVO5_POSITION = -1;

    @Override
    public void runOpMode() throws InterruptedException {
//        DcMotorEx motor0 = hardwareMap.get(DcMotorEx.class, "0");
//        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "1");
//        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "2");
//        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class, "3");

        LynxModule lynxModule = hardwareMap.getAll(LynxModule.class).get(0);
        DcMotorControllerEx motorController = hardwareMap.getAll(DcMotorControllerEx.class).get(0);
        ServoControllerEx servoController = hardwareMap.getAll(ServoControllerEx.class).get(0);

        waitForStart();

        while (opModeIsActive()) {
            motorController.setMotorPower(0, MOTOR0_POWER);
            motorController.setMotorPower(1, MOTOR1_POWER);
            motorController.setMotorPower(2, MOTOR2_POWER);
            motorController.setMotorPower(3, MOTOR3_POWER);
            if (SERVO0_POSITION != -1) servoController.setServoPosition(0, SERVO0_POSITION);
            if (SERVO1_POSITION != -1) servoController.setServoPosition(1, SERVO1_POSITION);
            if (SERVO2_POSITION != -1) servoController.setServoPosition(2, SERVO2_POSITION);
            if (SERVO3_POSITION != -1) servoController.setServoPosition(3, SERVO3_POSITION);
            if (SERVO4_POSITION != -1) servoController.setServoPosition(4, SERVO4_POSITION);
            if (SERVO5_POSITION != -1) servoController.setServoPosition(5, SERVO5_POSITION);

            telemetry.addData(lynxModule.getDeviceName() + " connection info", lynxModule.getConnectionInfo());
            telemetry.addData(lynxModule.getDeviceName() + " auxiliary voltage", lynxModule.getAuxiliaryVoltage(VoltageUnit.VOLTS) + " V");
            telemetry.addData(lynxModule.getDeviceName() + " input voltage", lynxModule.getInputVoltage(VoltageUnit.VOLTS) + " V");
            telemetry.addData(lynxModule.getDeviceName() + " current", lynxModule.getCurrent(CurrentUnit.AMPS) + " A");
            telemetry.addData(lynxModule.getDeviceName() + " temperature", lynxModule.getTemperature(TempUnit.CELSIUS) + " oC");
            telemetry.addData(motorController.getDeviceName() + " connection info", motorController.getConnectionInfo());
            telemetry.addData("motor " + motorController.getMotorType(0).getName() + " current", motorController.getMotorCurrent(0, CurrentUnit.AMPS) + " A");
            telemetry.addData("motor " + motorController.getMotorType(1).getName() + " current", motorController.getMotorCurrent(1, CurrentUnit.AMPS) + " A");
            telemetry.addData("motor " + motorController.getMotorType(2).getName() + " current", motorController.getMotorCurrent(2, CurrentUnit.AMPS) + " A");
            telemetry.addData("motor " + motorController.getMotorType(3).getName() + " current", motorController.getMotorCurrent(3, CurrentUnit.AMPS) + " A");
            telemetry.addData(servoController.getDeviceName() + " connection info", servoController.getConnectionInfo());
            telemetry.addData("servo 0 position", servoController.getServoPosition(0));
            telemetry.addData("servo 1 position", servoController.getServoPosition(1));
            telemetry.addData("servo 2 position", servoController.getServoPosition(2));
            telemetry.addData("servo 3 position", servoController.getServoPosition(3));
            telemetry.addData("servo 4 position", servoController.getServoPosition(4));
            telemetry.addData("servo 5 position", servoController.getServoPosition(5));
            telemetry.update();
        }
    }
}
