package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.subsystem.ServoStuff;

@Config
public class Arm {
//    public static double ARM_OFFSET = 0;
    public static double ARM_BACKWARD = .994;
    public static double ARM_FORWARD = .462;
    public static double INTAKE_SPEED = .8;
    public static double ARM_LENGTH = 250;

    public HardwareMap hardwareMap;
    public ServoImplEx arm;
    public CRServoImplEx intake;

    private double position = 0;
    private double target = .5 * (ARM_BACKWARD + ARM_FORWARD);
    private double intakePower = 0;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        arm = hardwareMap.get(ServoImplEx.class, "arm");
        intake = hardwareMap.get(CRServoImplEx.class, "in");

        arm.setPwmRange(ServoStuff.AxonMaxServo.servoModePwmRange);
        intake.setPwmRange(ServoStuff.GobildaSpeedServo.continuousModePwmRange);

        // no use for this since we use Range.scale from angle to position
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);  // change so positive is ccw/forward
    }

    //rad ccw
    public double getPosition() {
        return position;
    }
    public double getTarget() {
        return target;
    }
    public void setTarget(double angle) {
//        target = Range.clip(angle, -.5, Math.PI + .5);
//        // todo find acceptable arm positions, and also make it accessible
    }

    public void intake() {
        intakePower = INTAKE_SPEED;
    }
    public void outtake() {
        intakePower = -INTAKE_SPEED;
    }
    public void stoptake() {
        intakePower = 0;
    }

    public void update() {
//        arm.setPosition(Range.scale(target, 0, Math.PI, ARM_FORWARD, ARM_BACKWARD));
//        intake.setPower(intakePower);
    }

    //rad
    public static double angleToServo(double angle) {
        return Math.toDegrees(angle) / 300.;
    }

    //rad
    public static Vector2D intakePosition(double angle) {
        return new Vector2D(ARM_LENGTH * Math.cos(angle), ARM_LENGTH * Math.sin(angle));
    }
}
