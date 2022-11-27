package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

// This seems like a useless class?
public class IntakeOuttake {

    public HardwareMap hardwareMap;
    public Vertical vertical;
    public Horizontal horizontal;
    public Turret turret;
    public Arm arm;

    public Supplier<Pose2d> position;

    private double verticalTarget;
    private double horizontalTarget;
    private double turretTarget;

    public IntakeOuttake(HardwareMap hardwareMap, Supplier<Pose2d> position) {
        this.hardwareMap = hardwareMap;
        this.position = position;

        vertical = new Vertical(hardwareMap);
        horizontal = new Horizontal(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    // mm
    public void setVerticalTarget(double height) {
        verticalTarget = height;
    }
    public void adjustVerticalTarget(double height) {
        verticalTarget += height;
    }

    // mm
    public void setHorizontalTarget(double length) {
        horizontalTarget = length;
    }
    public void adjustHorizontalTarget(double length) {
        horizontalTarget += length;
    }

    // rad
    public void setTurretTarget(double angle) {
        turretTarget = angle;
    }
    public void adjustTurretTarget(double angle) {
        turretTarget += angle;
    }

    public void update() {
        vertical.update();
        horizontal.update();
        turret.update();
    }
}
