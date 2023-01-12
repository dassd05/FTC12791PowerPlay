package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    public HardwareMap hardwareMap;
    public DcMotorEx motor;
    public AnalogInput encoder;

    private double lastVoltage = -1;
    private int sector = 0;
    private double position = 0;
    private double target = 0;

    public Turret(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motor = hardwareMap.get(DcMotorEx.class, "tur");
        encoder = hardwareMap.get(AnalogInput.class, "tur");
    }

    public int getSector() {
        return sector;
    }
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
        double voltage = encoder.getVoltage() / encoder.getMaxVoltage();

        if (lastVoltage == -1) {
            lastVoltage = voltage;
            return;
        }

//        if (voltage > .9 && lastVoltage < .1) sector --;
//        if (voltage < .1 && lastVoltage > .9) sector ++;
        position = Math.toRadians((sector + voltage) * 72);
        lastVoltage = voltage;

        double error = Angle.normDelta(target - position);
        // todo motor pid

        // todo no accurate way to measure angle right now
        motor.setPower(target);
    }
}
