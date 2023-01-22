package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystem.MultiMotor;

@Config
public class Vertical {
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients();

    public HardwareMap hardwareMap;
    public DcMotorEx v2;
    public DcMotor v1, v3;

    private int position = 0;
    private int target = 0;

    public Vertical(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        v1 = hardwareMap.get(DcMotorEx.class, "v1");
        v2 = hardwareMap.get(DcMotorEx.class, "v3");
        v3 = hardwareMap.get(DcMotor.class, "v2");

        v1.setDirection(DcMotorEx.Direction.REVERSE);
        v1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        v2.setDirection(DcMotorSimple.Direction.REVERSE);
        v2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        v3.setDirection(DcMotorSimple.Direction.REVERSE);
        v3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        v2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        v1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        v3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        motors.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients()); // todo is this fine, or do we need our own pidf controller?
    }

    // mm
    public int getPosition() {
        return (position);
    }
    public double getTarget() {
        return ticksToMM(target);
    }
    public void setTarget(int target) {
        this.target = target;
    }

    public void update(int target) {
        v2.setTargetPosition(target);
        v2.setPower(.85);
        v1.setPower(v1.getPower());
        v3.setPower(v1.getPower());
    }

    public static double ticksToMM(int ticks) {
        return ticks;
    }
    public static int mmToTicks(double mm) {
        return (int) Math.round(mm);
    }
}
