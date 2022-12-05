package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

// todo This seems like a useless class?
public class IntakeOuttake {

    public HardwareMap hardwareMap;
    public Vertical vertical;
    public Horizontal horizontal;
    public Turret turret;
    public Arm arm;

    public Supplier<Pose2d> position;

    private double verticalTarget = 0;
    private double horizontalTarget = 0;
    private double turretTarget = 0;
    private double armTarget = 0;
    private boolean intaked = false;

    public IntakeOuttake(HardwareMap hardwareMap, Supplier<Pose2d> position) {
        this.hardwareMap = hardwareMap;
        this.position = position;

        vertical = new Vertical(hardwareMap);
        horizontal = new Horizontal(hardwareMap);
        turret = new Turret(hardwareMap);
        arm = new Arm(hardwareMap);
    }

    // mm
    public void setVerticalTarget(double height) {
        verticalTarget = height;
        vertical.setTarget(height);
    }
    public void adjustVerticalTarget(double height) {
        verticalTarget += height;
        vertical.setTarget(vertical.getTarget() + height);
    }

    // mm
    public void setHorizontalTarget(double length) {
        horizontalTarget = length;
        horizontal.setTarget(length);
    }
    public void adjustHorizontalTarget(double length) {
        horizontalTarget += length;
        horizontal.setTarget(horizontal.getTarget() + length);
    }

    // rad
    public void setTurretTarget(double angle) {
        turretTarget = angle;
        turret.setTarget(angle);
    }
    public void adjustTurretTarget(double angle) {
        turretTarget += angle;
        turret.setTarget(turret.getTarget() + angle);
    }

    // rad
    public void setArmTarget(double angle) {
        armTarget = angle;
        arm.setTarget(angle);
    }
    public void adjustArmTarget(double angle) {
        armTarget += angle;
        arm.setTarget(arm.getTarget() + angle);
    }
    public void intake() {
        if (!intaked) {
            intaked = true;
            arm.intake();
        }
    }
    public void outtake() {
        if (intaked) {
            intaked = false;
            arm.outtake();
        }
    }

    public void update() {
        vertical.update();
        horizontal.update();
        turret.update();
        arm.update();
    }
}
