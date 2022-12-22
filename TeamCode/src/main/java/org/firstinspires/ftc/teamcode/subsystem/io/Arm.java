package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.subsystem.ServoStuff;

@Config
public class Arm {
    public static double ARM_OFFSET = 0;
    public static double INTAKE_SPEED = 0;
    public static double ARM_LENGTH = 0;

    public HardwareMap hardwareMap;
    public ServoImplEx arm;
    public CRServoImplEx intake;

    private double position = 0;
    private double target = 0;
    private double intakePower = 0;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        arm = hardwareMap.get(ServoImplEx.class, "armLeft");
        intake = hardwareMap.get(CRServoImplEx.class, "armRight");

        arm.setPwmRange(ServoStuff.AxonMaxServo.servoModePwmRange);
        intake.setPwmRange(ServoStuff.GobildaTorqueServo.continuousModePwmRange);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);  // todo change so positive is ccw/forward
    }

    //rad ccw
    public double getPosition() {
        return position;
    }
    public double getTarget() {
        return target;
    }
    public void setTarget(double angle) {
        target = Angle.norm(angle);
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
        arm.setPosition(angleToServo(target) + ARM_OFFSET);
        intake.setPower(intakePower);
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
