package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MultiMotor implements DcMotorEx {
    public DcMotorEx[] motors;

    public MultiMotor(DcMotorEx... motors) {
        this.motors = motors;
    }

    public MultiMotor(HardwareMap hardwareMap, String... names) {
        motors = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i ++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
        }
    }


    @Override
    public void setMotorEnable() {
        for (DcMotorEx motor: motors) {
            motor.setMotorEnable();
        }
    }

    @Override
    public void setMotorDisable() {
        for (DcMotorEx motor: motors) {
            motor.setMotorDisable();
        }
    }

    @Override
    public boolean isMotorEnabled() {
        for (DcMotorEx motor: motors) {
            if (!motor.isMotorEnabled()) return false;
        }
        return true;
    }

    @Override
    public void setVelocity(double angularRate) {
        for (DcMotorEx motor: motors) {
            motor.setVelocity(angularRate);
        }
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        for (DcMotorEx motor: motors) {
            motor.setVelocity(angularRate, unit);
        }
    }

    @Override
    public double getVelocity() {
        double avg = 0;
        for (DcMotorEx motor: motors) {
            avg += motor.getVelocity();
        }
        return avg / 3;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        double avg = 0;
        for (DcMotorEx motor: motors) {
            avg += motor.getVelocity(unit);
        }
        return avg / 3;
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        for (DcMotorEx motor: motors) {
            motor.setPIDCoefficients(mode, pidCoefficients);
        }
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        for (DcMotorEx motor: motors) {
            motor.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        for (DcMotorEx motor: motors) {
            motor.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        for (DcMotorEx motor: motors) {
            motor.setPositionPIDFCoefficients(p);
        }
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motors[0].getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motors[0].getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        for (DcMotorEx motor: motors) {
            motor.setTargetPositionTolerance(tolerance);
        }
    }

    @Override
    public int getTargetPositionTolerance() {
        return motors[0].getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        double avg = 0;
        for (DcMotorEx motor: motors) {
            avg += motor.getCurrent(unit);
        }
        return avg / 3;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motors[0].getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        for (DcMotorEx motor: motors) {
            motor.setCurrentAlert(current, unit);
        }
    }

    @Override
    public boolean isOverCurrent() {
        for (DcMotorEx motor: motors) {
            if (motor.isOverCurrent()) return true;
        }
        return false;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motors[0].getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        for (DcMotorEx motor: motors) {
            motor.setMotorType(motorType);
        }
    }

    @Override
    public DcMotorController getController() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Use MultiMotor.getControllers() instead.");
    }

    public DcMotorController[] getControllers() {
        DcMotorController[] controllers = new DcMotorController[motors.length];
        for (int i = 0; i < motors.length; i ++) {
            controllers[i] = motors[i].getController();
        }
        return controllers;
    }

    @Override
    public int getPortNumber() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Use MultiMotor.getPortNumbers() instead.");
    }

    public int[] getPortNumbers() {
        int[] ports = new int[motors.length];
        for (int i = 0; i < motors.length; i ++) {
            ports[i] = motors[i].getPortNumber();
        }
        return ports;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motors[0].getZeroPowerBehavior();
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        for (DcMotorEx motor: motors) {
            motor.setPowerFloat();
        }
    }

    @Override
    public boolean getPowerFloat() {
        return motors[0].getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        for (DcMotorEx motor: motors) {
            motor.setTargetPosition(position);
        }
    }

    @Override
    public int getTargetPosition() {
        return motors[0].getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        for (DcMotorEx motor: motors) {
            if (motor.isBusy()) return true;
        }
        return false;
    }

    @Override
    public int getCurrentPosition() {
        int avg = 0;
        for (DcMotorEx motor: motors) {
            avg += motor.getCurrentPosition();
        }
        return avg / 3;    }

    @Override
    public void setMode(RunMode mode) {
        for (DcMotorEx motor: motors) {
            motor.setMode(mode);
        }
    }

    @Override
    public RunMode getMode() {
        return motors[0].getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        for (DcMotorEx motor: motors) {
            motor.setDirection(direction);
        }
    }

    public void setDirections(Direction... directions) {
        if (directions.length != motors.length) throw new IllegalArgumentException("directions must be the same length as the number of motors.");

        for (int i = 0; i < motors.length; i ++) {
            motors[i].setDirection(directions[i]);
        }
    }

    @Override
    public Direction getDirection() throws UnsupportedOperationException {
        // In case the directions are different. Too lazy to actually check if that's not the case.
        throw new UnsupportedOperationException("Use MultiMotor.getDirections() instead.");
    }

    public Direction[] getDirections() {
        Direction[] directions = new Direction[motors.length];
        for (int i = 0; i < motors.length; i ++) {
            directions[i] = motors[i].getDirection();
        }
        return directions;
    }

    @Override
    public void setPower(double power) {
        for (DcMotorEx motor: motors) {
            motor.setPower(power);
        }
    }

    @Override
    public double getPower() {
        double avg = 0;
        for (DcMotorEx motor: motors) {
            avg += motor.getPower();
        }
        return avg / 3;    }

    @Override
    public Manufacturer getManufacturer() {
        return motors[0].getManufacturer();
    }

    @Override
    public String getDeviceName() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Use MultiMotor.getDeviceNames() instead.");
    }

    public String[] getDeviceNames() {
        String[] names = new String[motors.length];
        for (int i = 0; i < motors.length; i ++) {
            names[i] = motors[i].getDeviceName();
        }
        return names;
    }

    @Override
    public String getConnectionInfo() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Use MultiMotor.getConnectionInfos() instead.");
    }

    public String[] getConnectionInfos() {
        String[] infos = new String[motors.length];
        for (int i = 0; i < motors.length; i ++) {
            infos[i] = motors[i].getConnectionInfo();
        }
        return infos;
    }

    @Override
    public int getVersion() {
        return motors[0].getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        for (DcMotorEx motor: motors) {
            motor.resetDeviceConfigurationForOpMode();
        }
    }

    @Override
    public void close() {
        for (DcMotorEx motor: motors) {
            motor.close();
        }
    }
}
