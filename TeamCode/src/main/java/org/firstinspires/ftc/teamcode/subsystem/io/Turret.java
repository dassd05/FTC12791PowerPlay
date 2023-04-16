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
    public static double ABSOLUTE_FORWARD = .87; // volts. ccw is decreasing
    //public static double ABSOLUTE_FORWARD = .89; // volts. ccw is decreasing
    public static PIDCoefficients PID = new PIDCoefficients(2.2, 7e-7, 0);
    public static double INTEGRAL_CAP = .55;


    public static PIDCoefficients PIDZeroed = new PIDCoefficients(.06, 8e-10, /*2000000*/ 0);

    public HardwareMap hardwareMap;
    public DcMotorEx motor;
    public AnalogInput absoluteEncoder;
    public Encoder quadratureEncoder;

    private double lastVoltage = -1;
//    private int sector = 0;
    private double position = 0;
    private double target = 0;
    public int pos = 0;

    private long lastTime = System.nanoTime();
    private double lastError = 0;
    public static double totalError = 0;
    private long totalTime = 0;

    private int targetPosition = 0;

    public static boolean within = false;

    public Turret(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motor = hardwareMap.get(DcMotorEx.class, "tur"); // positive power ccw
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "tur");
        quadratureEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "br")); // port 1

        // todo temp
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public Turret(HardwareMap hardwareMap, boolean zeroed) {
        this(hardwareMap);
        this.zeroed = zeroed;
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

    public int turretTargetPosition = 0;

    public double getTurretTargetPosition() {
        return turretTargetPosition;
    }
    public void setTurretTargetPosition(int target) {
        turretTargetPosition = target;
    }


    int offset = 0;

    boolean zeroed = false;

    public void setZero(boolean yes) {
        zeroed = yes;
    }
    public boolean zeroed () {
        return zeroed;
    }

    double power = 0;
    public void update() {
        if (!zeroed) {
            double voltage = absoluteEncoder.getVoltage();// / absoluteEncoder.getMaxVoltage();
            quadratureEncoder.update();
            //int pos = quadratureEncoder.getCurrentPosition();

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
            double error = -ABSOLUTE_FORWARD + voltage;
            if (error * lastError <= 0) totalError = 0;
            else totalError += (error - lastError) * (time - lastTime);
            double d = (error - lastError) / (time - lastTime);
            double i = totalError * PID.i;

            double power = PID.p * error + (Math.abs(i) < INTEGRAL_CAP ? i : Math.signum(i) * INTEGRAL_CAP) + PID.d * d;
            motor.setPower(power);

            if (/*Math.abs(error) <= .04 ||*/ (totalTime) > 3e9) {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                offset = quadratureEncoder.getCurrentPosition();
                zeroed = true;
            }
            totalTime += time - lastTime;
            lastTime = time;
            lastError = error;
        } else {
            quadratureEncoder.update();
            long time = System.nanoTime();
            pos = -quadratureEncoder.getCurrentPosition() + offset;

            double error = turretTargetPosition - pos;
            if (error * lastError <= 0) totalError = 0;
            else totalError += (error - lastError) * (time - lastTime);
            double d = (error - lastError) / (time - lastTime);
            double i = totalError * PIDZeroed.i;

            if (Math.abs(error) > 10)
                power = PIDZeroed.p * error + (Math.abs(i) < INTEGRAL_CAP ? i : Math.signum(i) * INTEGRAL_CAP) + PIDZeroed.d * d;
            else
                power = 0;

            within = Math.abs(error) < 75;

            //double power = PIDZeroed.p * error + (Math.abs(i) < INTEGRAL_CAP ? i : Math.signum(i) * INTEGRAL_CAP) + PIDZeroed.d * d;
            motor.setPower(power);
            lastTime = time;
            lastError = error;
        }
    }

    public static boolean getWithin() {
        return within;
    }

    //public static double power1;
    public void update(Telemetry telemetry) {
        if (!zeroed) {
            double voltage = absoluteEncoder.getVoltage();// / absoluteEncoder.getMaxVoltage();
            quadratureEncoder.update();
            //int pos = quadratureEncoder.getCurrentPosition();

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
            double error = -ABSOLUTE_FORWARD + voltage;
            if (error * lastError <= 0) totalError = 0;
            else totalError += (error - lastError) * (time - lastTime);
            double d = (error - lastError) / (time - lastTime);
            double i = totalError * PID.i;

            double power = PID.p * error + (Math.abs(i) < INTEGRAL_CAP ? i : Math.signum(i) * INTEGRAL_CAP) + PID.d * d;
            motor.setPower(power);

            if (/*Math.abs(error) <= .04 ||*/ (totalTime) > 3e9) {
                offset = quadratureEncoder.getCurrentPosition();
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                zeroed = true;
            }
            totalTime += time - lastTime;
            lastTime = time;
            lastError = error;
        } else {
            quadratureEncoder.update();
            long time = System.nanoTime();
            int pos = -quadratureEncoder.getCurrentPosition() + offset;

            double error = turretTargetPosition - pos;
            if (error * lastError <= 0) totalError = 0;
            else totalError += (error - lastError) * (time - lastTime);
            double d = (error - lastError) / (time - lastTime);
            double i = totalError * PIDZeroed.i;

            double power = PIDZeroed.p * error + (Math.abs(i) < INTEGRAL_CAP ? i : Math.signum(i) * INTEGRAL_CAP) + PIDZeroed.d * d;
            motor.setPower(power);
            lastTime = time;
            lastError = error;

            telemetry.addData("Error", error);
            telemetry.addData("P Error", .012 * error);
            telemetry.addData("I Error", (Math.abs(i) < INTEGRAL_CAP ? i : Math.signum(i) * INTEGRAL_CAP));
        }
    }

    public static double ticksToRadians(int ticks) {
        return ticks / 751.8 * 24 / 125 * Math.PI * 2;
    }
    public static int radiansToTicks(double radians) {
        return (int) Math.round(radians / Math.PI / 2 * 125 / 24 * 751.8);
    }
}
