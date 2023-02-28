package org.firstinspires.ftc.teamcode.subsystem.io;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.function.Supplier;

public class IntakeOuttake {
    public static double VERTICAL_OFFSET = 0; // mm slides are above ground

    // todo return a boolean on the functions: true for in range, and false for out of range. But should we still attempt to go to an out of range target?
    public HardwareMap hardwareMap;
    public Vertical vertical;
    public Horizontal horizontal;
    public Turret turret;
    public Arm arm;
    public RevColorSensorV3 colorSensor;

    public Supplier<Pose2d> position;

    // todo remove these local variables for targets and just use the object's getters? or keep as is?
    private int verticalTarget = 0;
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
//        colorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
    }

    // mm
    public void setVerticalTarget(int height) {
        height = (int) Math.max(height - VERTICAL_OFFSET, 0);
        verticalTarget = height;
        vertical.setTarget(height);
    }
    public void adjustVerticalTarget(int height) {
//        verticalTarget += height;
//        vertical.setTarget(vertical.getTarget() + height);
        setVerticalTarget(verticalTarget + height);
    }

    // mm
    public void setHorizontalTarget(double length) {
        horizontalTarget = length;
        horizontal.setTarget(length);
    }
    public void adjustHorizontalTarget(double length) {
//        horizontalTarget += length;
//        horizontal.setTarget(horizontal.getTarget() + length);
        setHorizontalTarget(horizontalTarget + length);
    }

    // rad
    public void setTurretTarget(double angle) {
        turretTarget = angle;
        turret.setTarget(angle);
    }
    public void adjustTurretTarget(double angle) {
//        turretTarget += angle;
//        turret.setTarget(turret.getTarget() + angle);
        setTurretTarget(turretTarget + angle);
    }

    // rad
    public void setArmTarget(double position) {
        armTarget = position;
    }
    public void adjustArmTarget(double angle) {
//        armTarget += angle;
//        arm.setTarget(arm.getTarget() + angle);
        setArmTarget(armTarget + angle);
    }

//    public void stoptake() {
//        arm.stoptake();
//    }

//    // WILL adjust vertical slides if we must go out of the arm's reach.
//    // adjust the arm head (cone)'s vertical position without moving slides or changing it's horizontal position
//    public void adjustVerticalNoSlides(double height) {  // do a setVerticalNoSlides? where it's absolute height instead of adjustment
//        Vector2D currentArmPosition = Arm.intakePosition(arm.getTarget());
//        double newHeight = currentArmPosition.getY() + height;
//        double newAngle;
//        if (newHeight < Arm.ARM_LENGTH) {
//            newAngle = Math.asin(newHeight / Arm.ARM_LENGTH);
//            newAngle = arm.getTarget() < Math.PI / 2 ? newAngle : Math.PI - newAngle;
//        } else {
//            newAngle = Math.PI / 2;
//            adjustVerticalTarget((int) (newHeight - Arm.ARM_LENGTH));
//        } // but what if it goes BELOW the lowest arm point?
//        setArmTarget(newAngle);
//        adjustHorizontalTarget(currentArmPosition.getX() - Arm.intakePosition(newAngle).getX());
//    }

    // idk how to name it
    // so intake (arm head) position stays fixed, while the rest of the IO system moves (so perhaps vertical lifts and horizontal moves)
    // but cone stays the same place
//    public void setArmStayStill(double angle) {
//        Vector2D currentArmPosition = Arm.intakePosition(arm.getTarget());
//        Vector2D newArmPosition = Arm.intakePosition(angle);
//
//        setArmTarget(angle);
//        adjustVerticalTarget((int) (currentArmPosition.getY() - newArmPosition.getY()));
//        adjustHorizontalTarget(currentArmPosition.getX() - newArmPosition.getX());
//    }
//    public void adjustArmStayStill(double angle) {
//        setArmStayStill(arm.getTarget() + angle);
//    }

    // as of now, just moves vertical height to target
    public void deploy(JunctionLevel junctionLevel) {
        setVerticalTarget((int) junctionLevel.getHeight());
    }

    // 1, 1 is back left. 5, 5 is front right
    public void deploy(int row, int col) {
        double height;

        if (row % 2 == 1 && col % 2 == 1) height = 0;
        else if (row % 2 == 0 && col % 2 == 0) height = 2;
        else if (Math.abs(row - 3) == 1 && Math.abs(col - 3) == 1) height = 3;
        else height = 1;

        double tileDistance = 24 * 25.4;
        setTargetAbsolute(col * tileDistance, row * tileDistance, height);
    }

    //relative, mm
    public void setTargetRelative(double x, double y, double z) {
        double angle = Math.atan2(x, y);
        double angleOffset = Angle.normDelta(angle - turret.getPosition());
        double flatDistance = Math.sqrt(x*x + y*y);
        boolean forward = Math.abs(angleOffset) < Math.PI * .5 || flatDistance > Horizontal.MAX_BACKWARD + Arm.ARM_LENGTH;

        setTurretTarget(Angle.norm(angle + (forward ? 0 : Math.PI)));

        double armAngle = Math.atan2(z, flatDistance);
        setArmTarget(Angle.norm(armAngle + (forward ? 0 : Math.PI)));

        setVerticalTarget((int) (z - Math.sin(armAngle) * Arm.ARM_LENGTH));
        setHorizontalTarget((flatDistance - Math.cos(armAngle) * Arm.ARM_LENGTH) * (forward ? 1 : -1));
    }

    public void setTargetAbsolute(double x, double y, double z) {
        setTargetRelative(x - position.get().getX(), y - position.get().getY(), z);
    }

    public void collapse() {
        setVerticalTarget(0);
        setHorizontalTarget(0);
//        setTurretTarget(0);
        setArmTarget(Math.PI / 2);
    }

    public void update() {
//        vertical.update();
//        horizontal.update();
        turret.update();
        //arm.update();
    }

    public enum JunctionLevel {
        NONE(VERTICAL_OFFSET),
        GROUND(0), // todo
        LOW(1),
        MID(2),
        HIGH(3);

        private final double height;

        JunctionLevel(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }
}
