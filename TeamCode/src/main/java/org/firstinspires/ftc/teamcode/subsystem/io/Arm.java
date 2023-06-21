package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@Config
public class Arm {
//    public static double ARM_OFFSET = 0;
    public static double ARM_INTAKE = .78;
    public static double ARM_REST = .36;
    public static double ARM_OUTTAKE = .22; //old .38
    public static double ARM_ANGLED = .30;

    public static double ARM_LENGTH = 250;
    public static double ARM_HORIZONTAL_LENGTH = 346.17;

    boolean open = true;

    public static double CLAW_OPEN = .35;
    public static double CLAW_CLOSE = .56;//.55;
    public static double CLAW_CAP = .53;

    public static double MECH_ALIGN_OPEN = .97;
    public static double MECH_ALIGN_CLOSE = .77;
    public static double MECH_ALIGN_PREP = .84;
    public static double MECH_ALIGN_EXTRA = .79;

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

    public boolean isAligning() {
        return aligner.getPosition() - MECH_ALIGN_CLOSE < (MECH_ALIGN_OPEN - MECH_ALIGN_CLOSE) / 2;
    }
    public void setAligner(boolean aligning) {
        if (aligning)
            aligner.setPosition(MECH_ALIGN_CLOSE);
        else
            aligner.setPosition(MECH_ALIGN_OPEN);
    }

    public void setAligner(boolean aligning, double time) {
        if (time > 1200) {
            if (aligning)
                aligner.setPosition(MECH_ALIGN_CLOSE);
            else
                aligner.setPosition(MECH_ALIGN_OPEN);
        }
        else
            aligner.setPosition(MECH_ALIGN_PREP);
    }

    public void setAligner(boolean aligning, double time, double cut) {
        if (time > cut) {
            if (aligning)
                aligner.setPosition(MECH_ALIGN_CLOSE);
            else
                aligner.setPosition(MECH_ALIGN_OPEN);
        }
        else
            aligner.setPosition(MECH_ALIGN_PREP);
    }

    public void setAligner(boolean aligning, double time, double cut, boolean extra) {
        if (time > cut) {
            if (aligning)
                aligner.setPosition(MECH_ALIGN_EXTRA);
            else
                aligner.setPosition(MECH_ALIGN_OPEN);
        }
        else
            aligner.setPosition(MECH_ALIGN_PREP);
    }


    // positive moves to more open
    public void adjustAligner(double amount) {
        aligner.setPosition(Range.clip(aligner.getPosition() + amount, MECH_ALIGN_CLOSE, MECH_ALIGN_OPEN));
    }
    public void adjustAlignerDegrees(double amount) {
        adjustAligner(amount * (MECH_ALIGN_OPEN - MECH_ALIGN_CLOSE) / 90);
    }


    public void update() {
        arm.setPosition(target);
        openClaw(open);
        intakeWrist(intakeWrist);
    }
}
