package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.SliderCrankLinkage;

@Config
public class Horizontal {
    // position at which horizontal is completely in (ie target = 0)
    public static double FORWARD1_OFFSET = 0;
    public static double FORWARD2_OFFSET = 0;
    public static double BACKWARD1_OFFSET = 0;
    public static double BACKWARD2_OFFSET = 0;

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

        // positive position = outwards
        forward1.setDirection(Servo.Direction.REVERSE);  // todo
        backward1.setDirection(Servo.Direction.REVERSE);

        forward = new SliderCrankLinkage(0, 0);
        backward = new SliderCrankLinkage(0, 0);

        // should we constrain the velocity of the linkage with a ServoEx class?
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
        // we just say that only one side is nonzero at any moment
        double forwardTarget = target > 0 ? forward.positionInv(target) : 0;
        double backwardTarget = target < 0 ? backward.positionInv(target) : 0;

        forward1.setPosition(angleToGobildaServo(forwardTarget) + FORWARD1_OFFSET);
        forward2.setPosition(angleToGobildaServo(forwardTarget) + FORWARD2_OFFSET);
        backward1.setPosition(angleToAxonServo(backwardTarget) + BACKWARD1_OFFSET);
        backward2.setPosition(angleToAxonServo(backwardTarget) + BACKWARD2_OFFSET);
    }

    //rad
    public static double angleToGobildaServo(double angle) {
        return Math.toDegrees(angle) / 300;
    }
    public static double angleToAxonServo(double angle) {
        return Math.toDegrees(angle) / 355;
    }
}
