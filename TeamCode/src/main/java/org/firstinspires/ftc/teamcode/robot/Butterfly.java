package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class Butterfly {

    public HardwareMap hardwareMap;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public Servo servoFrontLeft;
    public Servo servoFrontRight;
    public Servo servoBackLeft;
    public Servo servoBackRight;
    public BNO055IMU imu;

    public MecanumDrive mecanum;
    public TankDrive tank;

    private State state;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public Butterfly(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoFrontLeft = hardwareMap.get(Servo.class, "servoFrontLeft");
        servoFrontRight = hardwareMap.get(Servo.class, "servoFrontRight");
        servoBackLeft = hardwareMap.get(Servo.class, "servoBackLeft");
        servoBackRight = hardwareMap.get(Servo.class, "servoBackRight");

        mecanum = new MecanumDrive(0, 0, 0, 0, 0, 0) {
            @Override
            public void setMotorPowers(double fl, double bl, double br, double fr) {
                frontLeft.setPower(fl);
                backLeft.setPower(bl);
                backRight.setPower(br);
                frontRight.setPower(fr);
            }

            @NonNull
            @Override
            public List<Double> getWheelPositions() {
                return Arrays.asList(
                        ticksToInches(frontLeft.getCurrentPosition()),
                        ticksToInches(backLeft.getCurrentPosition()),
                        ticksToInches(backRight.getCurrentPosition()),
                        ticksToInches(frontRight.getCurrentPosition())
                );
            }

            @Override
            public List<Double> getWheelVelocities() {
                return Arrays.asList(
                        ticksToInches(frontLeft.getVelocity()),
                        ticksToInches(backLeft.getVelocity()),
                        ticksToInches(backRight.getVelocity()),
                        ticksToInches(frontRight.getVelocity())
                );
            }

            @Override
            protected double getRawExternalHeading() {
                return imu.getAngularOrientation().firstAngle;
            }

            public double ticksToInches(double ticks) {
                return ticks / 100;
            }
        };
        tank = new TankDrive(0, 0, 0, 0) {
            @Override
            public void setMotorPowers(double left, double right) {
                frontLeft.setPower(left);
                backLeft.setPower(left);
                backRight.setPower(right);
                frontRight.setPower(right);
            }

            @NonNull
            @Override
            public List<Double> getWheelPositions() {
                return Arrays.asList(
                        (ticksToInches(frontLeft.getCurrentPosition()) + ticksToInches(backLeft.getCurrentPosition())) * .5,
                        (ticksToInches(backRight.getCurrentPosition()) + ticksToInches(frontRight.getCurrentPosition())) * .5
                );
            }

            @Override
            public List<Double> getWheelVelocities() {
                return Arrays.asList(
                        (ticksToInches(frontLeft.getVelocity()) + ticksToInches(backLeft.getVelocity())) * .5,
                        (ticksToInches(backRight.getVelocity()) + ticksToInches(frontRight.getVelocity())) * .5
                );
            }

            @Override
            protected double getRawExternalHeading() {
                return imu.getAngularOrientation().firstAngle;
            }

            public double ticksToInches(double ticks) {
                return ticks / 50;
            }
        };
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void drive(double forward, double turn) {
        // scales motor power proportionally so it doesn't exceed 1
        double limiter = Math.max(Math.abs(forward) + Math.abs(turn), 1);

        frontLeftPower = (forward + turn) / limiter;
        frontRightPower = (forward - turn) / limiter;
        backLeftPower = (forward + turn) / limiter;
        backRightPower = (forward - turn) / limiter;
    }

    public void drive(double forward, double strafe, double turn) {
        strafe *= 1.1; // Counteract imperfect strafing maybe~~

        // scales motor power proportionally so it doesn't exceed 1
        double limiter = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        frontLeftPower = (forward + strafe + turn) / limiter;
        frontRightPower = (forward - strafe - turn) / limiter;
        backLeftPower = (forward - strafe + turn) / limiter;
        backRightPower = (forward + strafe - turn) / limiter;
    }

    public void driveFieldCentric(double forward, double strafe, double turn) {
        // Read inverse IMU heading, as the IMU heading is CW positive
        double heading = -imu.getAngularOrientation().firstAngle;

        double rotX = strafe * Math.cos(heading) - forward * Math.sin(heading);
        double rotY = strafe * Math.sin(heading) + forward * Math.cos(heading);

        drive(rotY, rotX, turn);
    }

    public void setServos(double position) {
        servoFrontLeft.setPosition(position);
        servoFrontRight.setPosition(position);
        servoBackLeft.setPosition(position);
        servoBackRight.setPosition(position);
    }

    public void update() {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // replace with arrow syntax if high enough java version :P
        switch (state) {
            case MECANUM: setServos(Constants.BUTTERFLY_IN); break;
            case TRACTION: setServos(Constants.BUTTERFLY_OUT); break;
            case STANDSTILL: setServos(Constants.BUTTERFLY_STILL); break;
        }
    }

    public enum State {
        MECANUM,
        TRACTION,
        STANDSTILL,
    }
}
