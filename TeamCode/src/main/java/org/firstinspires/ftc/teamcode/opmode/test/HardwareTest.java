package org.firstinspires.ftc.teamcode.opmode.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.function.Predicate;

@Config
@TeleOp(group = "test")
public class HardwareTest extends LinearOpMode {
    public static boolean CONTROL_HUB_SELECTED = false;
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

    public boolean controlHubLastSelected = CONTROL_HUB_SELECTED;

    public void resetValues() {
        MOTOR0_POWER = 0;
        MOTOR1_POWER = 0;
        MOTOR2_POWER = 0;
        MOTOR3_POWER = 0;
        SERVO0_POSITION = -1;
        SERVO1_POSITION = -1;
        SERVO2_POSITION = -1;
        SERVO3_POSITION = -1;
        SERVO4_POSITION = -1;
        SERVO5_POSITION = -1;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        //CONTROL_HUB_SELECTED = true;

        List<LynxModule> lynxModules = hardwareMap.getAll(LynxModule.class);
        List<LynxDcMotorController> motorControllers = hardwareMap.getAll(LynxDcMotorController.class);
        List<LynxServoController> servoControllers = hardwareMap.getAll(LynxServoController.class);

        assert lynxModules.size() == motorControllers.size() && lynxModules.size() == servoControllers.size();
        assert lynxModules.size() >= 1 && lynxModules.size() <= 2;

        boolean MUST_BE_CONTROL_HUB = lynxModules.size() == 1;

        LynxModule activeLynxModule = null;
        LynxDcMotorController activeMotorController = null;
        LynxServoController activeServoController = null;

        double[] motorPowers;
        double[] servoPositions;
        List<DcMotorEx> activeMotors = new ArrayList<>();
        List<ServoImplEx> activeServos = new ArrayList<>();

        List<AnalogInput> analogInputs = hardwareMap.getAll(AnalogInput.class);
        List<DigitalChannel> digitalChannels = hardwareMap.getAll(DigitalChannel.class);

        telemetry = TelemetryUtil.initTelemetry(telemetry);

        waitForStart();
        hardwareMap.logDevices();
        controlHubLastSelected = !CONTROL_HUB_SELECTED;

        while (opModeIsActive()) {
            if (MUST_BE_CONTROL_HUB) CONTROL_HUB_SELECTED = true;

            if (controlHubLastSelected != CONTROL_HUB_SELECTED) {
                controlHubLastSelected = CONTROL_HUB_SELECTED;
                resetValues();

                for (LynxModule lynxModule : lynxModules) {
                    if (LynxConstants.isEmbeddedSerialNumber(lynxModule.getSerialNumber()) == CONTROL_HUB_SELECTED) {
                        activeLynxModule = lynxModule;
                        break;
                    }
                }
                for (LynxDcMotorController motorController : motorControllers) {
                    if (LynxConstants.isEmbeddedSerialNumber(motorController.getSerialNumber()) == CONTROL_HUB_SELECTED) {
                        activeMotorController = motorController;
                        break;
                    }
                }
                for (LynxServoController servoController : servoControllers) {
                    if (LynxConstants.isEmbeddedSerialNumber(servoController.getSerialNumber()) == CONTROL_HUB_SELECTED) {
                        activeServoController = servoController;
                        break;
                    }
                }

                activeMotors.clear();
                activeServos.clear();
                for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) if (motor.getController() == activeMotorController) activeMotors.add(motor);
                for (ServoImplEx servo : hardwareMap.getAll(ServoImplEx.class)) if (servo.getController() == activeServoController) activeServos.add(servo);
            }

            motorPowers = new double[]{ MOTOR0_POWER, MOTOR1_POWER, MOTOR2_POWER, MOTOR3_POWER };
            servoPositions = new double[]{ SERVO0_POSITION, SERVO1_POSITION, SERVO2_POSITION, SERVO3_POSITION, SERVO4_POSITION, SERVO5_POSITION };

            assert activeLynxModule != null && activeMotorController != null && activeServoController != null;

            for (DcMotorEx motor : activeMotors) motor.setPower(motorPowers[motor.getPortNumber()]);
            for (ServoImplEx servo : activeServos) if (servoPositions[servo.getPortNumber()] == -1) servo.setPwmDisable(); else servo.setPosition(servoPositions[servo.getPortNumber()]);


            telemetry.addData("currently operating hub", (CONTROL_HUB_SELECTED ? "control hub " : "expansion hub ") + formatName(activeLynxModule));
            telemetry.addData(formatName(activeLynxModule) + " connection info", activeLynxModule.getConnectionInfo());
            telemetry.addData(formatName(activeLynxModule) + " auxiliary voltage (volts)", activeLynxModule.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            telemetry.addData(formatName(activeLynxModule) + " input voltage (volts)", activeLynxModule.getInputVoltage(VoltageUnit.VOLTS));  // uses the EXACT SAME LynxCommand as the one in VoltageSensor
            telemetry.addData(formatName(activeLynxModule) + " current (amps)", activeLynxModule.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(formatName(activeLynxModule) + " I2C current (amps)", activeLynxModule.getI2cBusCurrent(CurrentUnit.AMPS));
            telemetry.addData(formatName(activeLynxModule) + " temperature (°C)", activeLynxModule.getTemperature(TempUnit.CELSIUS));
            telemetry.addData(formatName(activeMotorController) + " connection info", activeMotorController.getConnectionInfo());
            for (DcMotorEx motor : activeMotors) {
//                telemetry.addLine(String.format("motor %d %s", motor.getPortNumber(), formatName(motor)))
                telemetry.addData(String.format("motor %d %s current (amps)", motor.getPortNumber(), formatName(motor)), motor.getCurrent(CurrentUnit.AMPS));
                telemetry.addData(String.format("motor %d %s position (ticks)", motor.getPortNumber(), formatName(motor)), motor.getCurrentPosition());
                telemetry.addData(String.format("motor %d %s velocity (ticks/sec)", motor.getPortNumber(), formatName(motor)), motor.getVelocity());
            }
            telemetry.addData(formatName(activeServoController) + " connection info", activeServoController.getConnectionInfo());
            for (ServoImplEx servo : activeServos) telemetry.addData(String.format("servo %d %s position", servo.getPortNumber(), formatName(servo)), servo.getPosition());
            telemetry.addData("Servo current owo", servoCurrent(activeLynxModule));
            for (AnalogInput analogInput : analogInputs) telemetry.addData(String.format("analog input %s (volts)", formatName(analogInput)), analogInput.getVoltage());
            for (DigitalChannel digitalChannel : digitalChannels) telemetry.addData(String.format("digital channel %s state", formatName(digitalChannel)), digitalChannel.getState());
            telemetry.update();
        }
    }

    public String formatName(HardwareDevice hardwareDevice) {
        return String.format("<code>%s</code> \"%s\"", hardwareDevice.getDeviceName(), hardwareMap.getNamesOf(hardwareDevice).iterator().next());
    }

    public static double servoCurrent(LynxModule lynxModule) throws InterruptedException {
        try {
            return CurrentUnit.MILLIAMPS.toAmps(new LynxGetADCCommand(lynxModule, LynxGetADCCommand.Channel.SERVO_CURRENT, LynxGetADCCommand.Mode.ENGINEERING).sendReceive().getValue());// + " A";
        } catch (LynxNackException e) {
            return -1;
//            return "Error: " + e.getMessage() + " - " + e.getNack().getNackReasonCodeAsEnum().name();
        }
    }
}
