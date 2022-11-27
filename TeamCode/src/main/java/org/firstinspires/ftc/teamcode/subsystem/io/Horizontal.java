package org.firstinspires.ftc.teamcode.subsystem.io;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.SliderCrankLinkage;

public class Horizontal {
    public HardwareMap hardwareMap;
    public Servo forward1;
    public Servo forward2;
    public Servo backward1;
    public Servo backward2;

    public SliderCrankLinkage forward;
    public SliderCrankLinkage backward;

    private double position = 0;
    private double target = 0;

    public Horizontal(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        forward1 = hardwareMap.get(Servo.class, "forward1");
        forward2 = hardwareMap.get(Servo.class, "forward2");
        backward1 = hardwareMap.get(Servo.class, "backward1");
        backward2 = hardwareMap.get(Servo.class, "backward2");

        forward = new SliderCrankLinkage(0, 0);
        backward = new SliderCrankLinkage(0, 0);

        // should we add pid to this system?
    }

    // mm
    public double getPosition() {
        return position;
    }
    public double getTarget() {
        return target;
    }
    public void setTarget(double target) {
        this.target = target;
    }

    public void update() {
        double forwardTarget = target > 0 ? forward.positionInv(target) : 0;
        double backwardTarget = target < 0 ? backward.positionInv(target) : 0;

    }
}
