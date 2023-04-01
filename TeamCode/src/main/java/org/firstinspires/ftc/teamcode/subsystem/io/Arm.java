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
    public static double ARM_INTAKE = .78;
    public static double ARM_REST = .36;
    public static double ARM_OUTTAKE = .22; //old .38
    public static double ARM_ANGLED = .31;

    public static double ARM_LENGTH = 250;
    public static double ARM_HORIZONTAL_LENGTH = 346.17;

    boolean open = true;

    public static double CLAW_OPEN = .37;
    public static double CLAW_CLOSE = .663;

    boolean intakeWrist = true;

    public static double WRIST_INTAKE = .715;
    public static double WRIST_OUTTAKE = .055;
    public static double WRIST_SAFE = 1;

    public HardwareMap hardwareMap;
    public ServoImplEx arm;
    public ServoImplEx claw;
    public ServoImplEx wrist;

    private double position = 0;
    private double target = 0;


    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        arm = hardwareMap.get(ServoImplEx.class, "arm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");

        //arm.setPwmRange(ServoStuff.AxonMaxServo.servoModePwmRange);
        //claw.setPwmRange(ServoStuff.GobildaSpeedServo.servoModePwmRange);
        //wrist.setPwmRange(ServoStuff.GobildaSpeedServo.servoModePwmRange);
    }

    public double getArmPosition() {
        return position;
    }
    public double getArmTarget() {
        return target;
    }
    public double adjust = 0.0;
    public void setArmTarget(double position) {
        target = Range.scale(Range.clip(position + adjust, 0, 1), 0,1, ARM_INTAKE, ARM_OUTTAKE);
        // todo find acceptable arm positions, and also make it accessible
    }
    public void adjustArm (double adjustment) {
        adjust = adjustment;
    }
    public void resetAdjustArm() {
        adjust = 0.0;
    }


    public void changeClaw() {
        open = !open;
    }
    public void openClaw(boolean yes) {
        if (yes)
            claw.setPosition(CLAW_OPEN);
        else
            claw.setPosition(CLAW_CLOSE);
    }

    public void flipWrist() {
        intakeWrist = !intakeWrist;
    }
    public void intakeWrist(boolean yes) {
        if (yes)
            wrist.setPosition(WRIST_INTAKE);
        else
            wrist.setPosition(WRIST_OUTTAKE);
    }


    public void update() {
        arm.setPosition(target);
        openClaw(open);
        intakeWrist(intakeWrist);
    }
}
