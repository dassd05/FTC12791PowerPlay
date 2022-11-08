package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deployment {

    public HardwareMap hardwareMap;
    public MultiMotor vertical;
    public Servo forward1;
    public Servo forward2;
    public Servo backward1;
    public Servo backward2;

    private Height height;
    private int verticalTarget;
    private double horizontalTarget;

    public Deployment(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        vertical = new MultiMotor(hardwareMap, "vertical1", "vertical2");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setTargetPosition(0);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        forward1 = hardwareMap.get(Servo.class, "forward1");
        forward2 = hardwareMap.get(Servo.class, "forward2");
        backward1 = hardwareMap.get(Servo.class, "backward1");
        backward2 = hardwareMap.get(Servo.class, "backward2");
    }

    public Height getHeight() {
        return height;
    }

    public void setHeight(Height height) {
        this.height = height;
        // verticalTarget = ...
        // linear and straight forward. just need to do pid
    }

    public void setHorizontalTarget(double target) {
        horizontalTarget = target;
        // need to calculate crank slider for distance to servo position
        // also will be multiple possible values due to two separate linkages
    }

    public void update() {
        vertical.setTargetPosition(verticalTarget);
    }

    public enum Height {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }
}
