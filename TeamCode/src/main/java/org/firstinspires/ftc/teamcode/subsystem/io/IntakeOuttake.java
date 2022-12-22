package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

// todo This seems like a useless class?
public class IntakeOuttake {

    public HardwareMap hardwareMap;
    public Vertical vertical;
    public Horizontal horizontal;
    public Turret turret;
    public Arm arm;
    public RevColorSensorV3 colorSensor;

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
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
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

    public void deploy(int row, int col) {
        double height;

        if (row % 2 == 1 && col % 2 == 1) height = 0;
        else if (row % 2 == 0 && col % 2 == 0) height = 2;
        else if (Math.abs(row - 3) == 1 && Math.abs(col - 3) == 1) height = 3;
        else height = 1;


    }

    //relative, mm
    public void setTarget(double x, double y, double z) {
        double angle = Math.atan2(x, y);
        double angleOffset = Angle.normDelta(angle - turret.getPosition());
        double flatDistance = Math.sqrt(x*x + y*y);
        boolean forward = Math.abs(angleOffset) < Math.PI * .5 || flatDistance > Horizontal.MAX_BACKWARD + Arm.ARM_LENGTH;

        setTurretTarget(Angle.norm(angle + (forward ? 0 : Math.PI)));

        double armAngle = Math.atan2(z, flatDistance);
        setArmTarget(Angle.norm(armAngle + (forward ? 0 : Math.PI)));

        setVerticalTarget(z - Math.sin(armAngle) * Arm.ARM_LENGTH);
        setHorizontalTarget((flatDistance - Math.cos(armAngle) * Arm.ARM_LENGTH) * (forward ? 1 : -1));
    }

    public void update() {
        vertical.update();
        horizontal.update();
        turret.update();
        arm.update();
    }
}
