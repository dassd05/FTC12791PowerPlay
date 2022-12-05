package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystem.MultiMotor;

@Config
public class Vertical {
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients();

    public HardwareMap hardwareMap;
    public MultiMotor motors;

    private int position = 0;
    private int target = 0;

    public Vertical(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        motors = new MultiMotor(hardwareMap, "vertical1", "vertical2", "vertical3");
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setTargetPosition(0);
        motors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients()); // is this fine, or do we need our own pidf?
    }

    // mm
    public double getPosition() {
        return ticksToMM(position);
    }
    public double getTarget() {
        return ticksToMM(target);
    }
    public void setTarget(double target) {
        this.target = mmToTicks(target);
    }

    public void update() {
        position = motors.getCurrentPosition();
        motors.setTargetPosition(target);
    }

    public static double ticksToMM(int ticks) {
        return ticks;
    }
    public static int mmToTicks(double mm) {
        return (int) Math.round(mm);
    }
}
