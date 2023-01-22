package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Turret {
    public static double ABSOLUTE_FORWARD = .89; // volts. ccw is decreasing
    public static PIDCoefficients PID = new PIDCoefficients(1.3, 2e-9, 0);
    public static double INTEGRAL_CAP = .55;

    public HardwareMap hardwareMap;
    public DcMotorEx motor;
    public AnalogInput absoluteEncoder;
    public Encoder quadratureEncoder;

    private double lastVoltage = -1;
//    private int sector = 0;
    private double position = 0;
    private double target = 0;

    private long lastTime = System.nanoTime();
    private double lastError = 0;
    private double totalError = 0;

    private int targetPosition = 0;

    public Turret(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motor = hardwareMap.get(DcMotorEx.class, "tur"); // positive power ccw
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "tur");
        quadratureEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fr")); // port 1

        // todo temp
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    public int getSector() {
//        return sector;
//    }
    //rad ccw
    public double getPosition() {
        return position;
    }
    public double getTarget() {
        return target;
    }
    public void setTarget(double angle) {
        target = Angle.normDelta(angle);
        // todo change back to Angle.norm and not normDelta
    }

    public void update() {
        double voltage = absoluteEncoder.getVoltage();// / absoluteEncoder.getMaxVoltage();
        quadratureEncoder.update();
        int pos = quadratureEncoder.getCurrentPosition();

//        if (lastVoltage == -1) {
//            lastVoltage = voltage;
//            return;
//        }

//        if (voltage > .9 && lastVoltage < .1) sector --;
//        if (voltage < .1 && lastVoltage > .9) sector ++;
//        position = Math.toRadians((sector + voltage) * 72);
//        lastVoltage = voltage;
//
//        double error = Angle.normDelta(target - position);
        // todo motor pid

        // todo no accurate way to measure angle right now
//        motor.setPower(target);

        // todo temp
        long time = System.nanoTime();
        double error = ABSOLUTE_FORWARD - voltage;
        if (error < 0 != lastError < 0) totalError = 0;
        else totalError += (error - lastError) * (time - lastTime);
        double d = (error - lastError) / (time - lastTime);
        double i = totalError * PID.i;

        double power = PID.p * error + (Math.abs(i) < INTEGRAL_CAP ? i : Math.signum(i) * INTEGRAL_CAP) + PID.d * d;
        motor.setPower(power);
        lastTime = time;
        lastError = error;
    }

    public void update(Telemetry telemetry) {
        double voltage = absoluteEncoder.getVoltage();// / absoluteEncoder.getMaxVoltage();
        long time = System.nanoTime();
        double error = ABSOLUTE_FORWARD - voltage;
        if (error < 0 != lastError < 0) totalError = 0;
        else totalError += (error) * (time - lastTime);
        double d = (error - lastError) / (time - lastTime);

        double pError = PID.p * error;
        double i = totalError * PID.i;
        double iError = (Math.abs(i) < INTEGRAL_CAP ? i : Math.signum(i) * INTEGRAL_CAP);
        double dError = PID.d * d;
        motor.setPower(pError + iError + dError);
        lastTime = time;
        lastError = error;

        telemetry.addData("Analog input", voltage);
        telemetry.addData("Error", error);
        telemetry.addData("P Error", pError);
        telemetry.addData("I Error", iError);
    }
}
