package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystem.ServoStuff;
import org.firstinspires.ftc.teamcode.subsystem.SliderCrankLinkage;

@Config
public class Horizontal {
    // position at which horizontal is completely in (ie target = 0)
//    public static double FORWARD1_OFFSET = 0;
//    public static double FORWARD2_OFFSET = 0;
//    public static double BACKWARD1_OFFSET = 0;
//    public static double BACKWARD2_OFFSET = 0;

    public static double FORWARD_LEFT_IN = .772; // diff = .667
    public static double FORWARD_LEFT_OUT = .105;
    public static double FORWARD_RIGHT_IN = .085; // diff = .665
    public static double FORWARD_RIGHT_OUT = .75;
    public static double BACKWARD_LEFT_IN = .23; // diff = .555
    public static double BACKWARD_LEFT_OUT = .785;
    public static double BACKWARD_RIGHT_IN = .99; // diff = .565
    public static double BACKWARD_RIGHT_OUT = .425;

    public static final double MAX_FORWARD = 2 * 360;
    public static final double MAX_BACKWARD = 2 * 240;

    public HardwareMap hardwareMap;
    public ServoImplEx forwardLeft;
    public ServoImplEx forwardRight;
    public ServoImplEx backwardLeft;
    public ServoImplEx backwardRight;

    public SliderCrankLinkage forward;
    public SliderCrankLinkage backward;

    private double position = 0;
    private double target = 0;

    public Horizontal(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        forwardLeft = hardwareMap.get(ServoImplEx.class, "hfl");
        forwardRight = hardwareMap.get(ServoImplEx.class, "hfr");
        backwardLeft = hardwareMap.get(ServoImplEx.class, "hbl");
        backwardRight = hardwareMap.get(ServoImplEx.class, "hbr");

        forwardLeft.setPwmRange(ServoStuff.AxonMiniServo.servoModePwmRange);
        forwardRight.setPwmRange(ServoStuff.AxonMiniServo.servoModePwmRange);
        backwardLeft.setPwmRange(ServoStuff.AxonMaxServo.servoModePwmRange);
        backwardRight.setPwmRange(ServoStuff.AxonMaxServo.servoModePwmRange);

        // positive position = outwards
//        forward1.setDirection(Servo.Direction.REVERSE);  // no use for this since we use Range.scale from angle to position
//        backward1.setDirection(Servo.Direction.REVERSE);

//        forward = new SliderCrankLinkage(360, 383.228);
//        backward = new SliderCrankLinkage(240, 277.684);
//        forward.calculateInverses(.001);  // ~6000 (.001 in 2pi) * 4 (p, v, a, t) * 2 (f + b) = 48000 (hard - p, v, a, t) calculations
//        backward.calculateInverses(.001);  // 4800 * 8 (bytes per double) * 2 (x + y) = 384000 bytes

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
        this.target = Range.clip(target, -MAX_BACKWARD, MAX_FORWARD);
    }

    public void update() {
//        // we just say that only one side is nonzero at any moment
//        double forwardTarget = target > 0 ? Math.PI - forward.positionInv(target + forward.rod - forward.crank).get(0) : 0; // if there are multiple values of the inverse, then we grab the lowest one
//        double backwardTarget = target < 0 ? Math.PI - backward.positionInv(-target + backward.rod - backward.crank).get(0) : 0; // thus it'll most likely be in the range [0, pi] todo
//
//        // limit range so no hitting or weird behavior
//        forwardTarget = Range.clip(forwardTarget, .13, 3);
//        backwardTarget = Range.clip(backwardTarget, .13, 3);
//
////        forward1.setPosition(angleToAxonServo(forwardTarget) + FORWARD1_OFFSET);
////        forward2.setPosition(angleToAxonServo(forwardTarget) + FORWARD2_OFFSET);
////        backward1.setPosition(angleToAxonServo(backwardTarget) + BACKWARD1_OFFSET);
////        backward2.setPosition(angleToAxonServo(backwardTarget) + BACKWARD2_OFFSET);
//        // ensure that we *NEVER* go past [0, pi]
//        forwardLeft.setPosition(clipScale(forwardTarget, 0, Math.PI, FORWARD_LEFT_IN, FORWARD_LEFT_OUT));
//        forwardRight.setPosition(clipScale(forwardTarget, 0, Math.PI, FORWARD_RIGHT_IN, FORWARD_RIGHT_OUT));
//        backwardLeft.setPosition(clipScale(backwardTarget, 0, Math.PI, BACKWARD_LEFT_IN, BACKWARD_LEFT_OUT));
//        backwardRight.setPosition(clipScale(backwardTarget, 0, Math.PI, BACKWARD_RIGHT_IN, BACKWARD_RIGHT_OUT));
    }

    // cannot rely on angle to position conversions, since the servos apparently don't have the range they report they do
    //rad
    public static double angleToGobildaServo(double angle) {
        return Math.toDegrees(angle) / 300;
    }
    public static double angleToAxonServo(double angle) {
        return Math.toDegrees(angle) / 355;
    }

    private static double clipScale(double n, double a1, double a2, double b1, double b2) {
        // scale then clip in the SLIGHTEST chance of a floating point error and it extends outside the given range
        return clip(Range.scale(n, a1, a2, b1, b2), b1, b2);
    }
    private static double clip(double n, double a, double b) {
        if (a < b) return Math.min(Math.max(n, a), b);
        else return Math.min(Math.max(n, b), a);
    }
}
