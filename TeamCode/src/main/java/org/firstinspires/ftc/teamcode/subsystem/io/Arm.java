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
    public static double ARM_ANGLED = .33;

    public static double ARM_LENGTH = 250;
    public static double ARM_HORIZONTAL_LENGTH = 346.17;

    boolean open = true;

    public static double CLAW_OPEN = .35;
    public static double CLAW_CLOSE = .56;

    public static double MECH_ALIGN_OPEN = .72;
    public static double MECH_ALIGN_CLOSE = .52;

    boolean intakeWrist = true;

    public static double WRIST_INTAKE = .02;
    public static double WRIST_OUTTAKE = .69;
    public static double WRIST_SAFE = .35;

    public HardwareMap hardwareMap;
    public ServoImplEx arm;
    public ServoImplEx claw;
    public ServoImplEx wrist;
    public ServoImplEx aligner;

    private double position = 0;
    private double target = 0;


    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        arm = hardwareMap.get(ServoImplEx.class, "arm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        aligner = hardwareMap.get(ServoImplEx.class, "align");

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

    public void setAligner(boolean aligning) {
        if (aligning)
            aligner.setPosition(MECH_ALIGN_CLOSE);
        else
            aligner.setPosition(MECH_ALIGN_OPEN);
    }


    public void update() {
        arm.setPosition(target);
        openClaw(open);
        intakeWrist(intakeWrist);
    }
}
