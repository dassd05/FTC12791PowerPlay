package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    public static double LEFT_OFFSET = 0;
    public static double RIGHT_OFFSET = 0;
    public static double INTAKE_AMOUNT = 0;

    public HardwareMap hardwareMap;
    public Servo left;
    public Servo right;

    private double position = 0;
    private double target = 0;
    private double intake = 0;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        left = hardwareMap.get(Servo.class, "armLeft");
        right = hardwareMap.get(Servo.class, "armRight");
        right.setDirection(Servo.Direction.REVERSE);  // todo change so positive is ccw/forward
    }

    //rad ccw
    public double getPosition() {
        return position;
    }
    public double getTarget() {
        return target;
    }
    public void setTarget(double angle) {
        target = angle;
    }

    public void intake() {
        intake += INTAKE_AMOUNT;
    }
    public void outtake() {
        intake -= INTAKE_AMOUNT;
    }

    public void update() {
        left.setPosition(angleToServo(target) + LEFT_OFFSET + INTAKE_AMOUNT);
        right.setPosition(angleToServo(target) + RIGHT_OFFSET);
    }

    //rad
    public static double angleToServo(double angle) {
        return Math.toDegrees(angle) / 300.;
    }
}
