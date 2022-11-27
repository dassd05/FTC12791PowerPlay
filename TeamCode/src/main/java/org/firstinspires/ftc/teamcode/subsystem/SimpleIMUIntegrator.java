package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * Simple Acceleration Integrator using Euler integration. Highly inaccurate.
 */
public class SimpleIMUIntegrator implements BNO055IMU.AccelerationIntegrator {
    protected BNO055IMU.Parameters parameters;
    protected Acceleration acceleration;
    protected Velocity velocity;
    protected Position position;

    @Override public void initialize(@NonNull BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
        this.parameters = parameters;
        position = initialPosition != null ? initialPosition.toUnit(DistanceUnit.METER) : new Position(DistanceUnit.METER, 0, 0, 0, System.nanoTime());
        velocity = initialVelocity != null ? initialVelocity.toUnit(DistanceUnit.METER) : new Velocity(DistanceUnit.METER, 0, 0, 0, System.nanoTime());
    }

    @Override public Position getPosition() { return position; }
    @Override public Velocity getVelocity() { return velocity; }
    @Override public Acceleration getAcceleration() { return this.acceleration; }

    @Override public void update(Acceleration linearAcceleration) {
        if (linearAcceleration.acquisitionTime == 0) return;

        acceleration = linearAcceleration;
        long time = acceleration.acquisitionTime;

        velocity.xVeloc += acceleration.xAccel * (time - velocity.acquisitionTime) * 1E-9;
        velocity.yVeloc += acceleration.yAccel * (time - velocity.acquisitionTime) * 1E-9;
        velocity.zVeloc += acceleration.zAccel * (time - velocity.acquisitionTime) * 1E-9;
        velocity.acquisitionTime = time;
        position.x += velocity.xVeloc * (time - position.acquisitionTime) * 1E-9;
        position.y += velocity.yVeloc * (time - position.acquisitionTime) * 1E-9;
        position.z += velocity.zVeloc * (time - position.acquisitionTime) * 1E-9;
        position.acquisitionTime = time;
    }
}
