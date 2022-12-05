package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import java.util.function.Supplier;

@Config
public class Butterfly {

    public static double FRONT_LEFT_IN = 0;
    public static double FRONT_LEFT_OUT = 0;
    public static double FRONT_LEFT_SS = 0;
    public static double BACK_LEFT_IN = 0;
    public static double BACK_LEFT_OUT = 0;
    public static double BACK_LEFT_SS = 0;
    public static double BACK_RIGHT_IN = 0;
    public static double BACK_RIGHT_OUT = 0;
    public static double BACK_RIGHT_SS = 0;
    public static double FRONT_RIGHT_IN = 0;
    public static double FRONT_RIGHT_OUT = 0;
    public static double FRONT_RIGHT_SS = 0;
    public static double IN_OUT_DISTANCE = 0;
    public static double IN_STILL_DISTANCE = 0;


    public HardwareMap hardwareMap;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx frontRight;
    public Servo servoFrontLeft;
    public Servo servoBackLeft;
    public Servo servoBackRight;
    public Servo servoFrontRight;
    public BNO055IMU imu;

    public SimpleMecanumDrive mecanum;
    public SimpleTankDrive tank;

    public Supplier<Pose2d> position;

    private State state;

    private double frontLeftPower;
    private double backLeftPower;
    private double backRightPower;
    private double frontRightPower;

    public Butterfly(HardwareMap hardwareMap, Supplier<Pose2d> position) {
        this.hardwareMap = hardwareMap;
        this.position = position;

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoFrontLeft = hardwareMap.get(Servo.class, "servoFrontLeft");
        servoBackLeft = hardwareMap.get(Servo.class, "servoBackLeft");
        servoBackRight = hardwareMap.get(Servo.class, "servoBackRight");
        servoFrontRight = hardwareMap.get(Servo.class, "servoFrontRight");

        mecanum = new SimpleMecanumDrive(0, 0, 0, 0, 0, 0);  // todo look at cad
        tank = new SimpleTankDrive(0, 0, 0, 0);
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void setMotorPowers(List<Double> powers) {
        assert powers.size() == 4;  // keep?
        frontLeftPower = powers.get(0);
        backLeftPower = powers.get(1);
        backRightPower = powers.get(2);
        frontRightPower = powers.get(3);
    }

    public void drive(double forward, double turn) {
        if (getState() == State.STANDSTILL) brake();
        else if (getState() == State.MECANUM) drive(forward, 0, turn);
        else setMotorPowers(tank.setDrivePower(new Pose2d(forward, 0, turn)));

//        // scales motor power proportionally so it doesn't exceed 1
//        double limiter = Math.max(Math.abs(forward) + Math.abs(turn), 1);
//
//        frontLeftPower = (forward + turn) / limiter;
//        backLeftPower = (forward + turn) / limiter;
//        backRightPower = (forward - turn) / limiter;
//        frontRightPower = (forward - turn) / limiter;
    }

    public void drive(double forward, double strafe, double turn) {
        if (getState() != State.MECANUM) drive(forward, turn);
        else setMotorPowers(mecanum.setDrivePower(new Pose2d(forward, strafe, turn)));

//        strafe *= 1.1; // Counteract imperfect strafing maybe~~
//
//        // scales motor power proportionally so it doesn't exceed 1
//        double limiter = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
//
//        frontLeftPower = (forward + strafe + turn) / limiter;
//        backLeftPower = (forward - strafe + turn) / limiter;
//        backRightPower = (forward + strafe - turn) / limiter;
//        frontRightPower = (forward - strafe - turn) / limiter;
    }

    public void driveFieldCentric(double forward, double strafe, double turn) {
        // Read inverse IMU heading, as the IMU heading is CW positive
        double heading = -imu.getAngularOrientation().firstAngle;
        double rotX = strafe * Math.cos(heading) - forward * Math.sin(heading);
        double rotY = strafe * Math.sin(heading) + forward * Math.cos(heading);

        drive(rotY, rotX, turn);
    }

    public void brake() {
        frontLeftPower = 0;
        backLeftPower = 0;
        backRightPower = 0;
        frontRightPower = 0;
    }

    public void setServos(List<Double> positions) {
        assert positions.size() == 4;
        servoFrontLeft.setPosition(positions.get(0));
        servoBackLeft.setPosition(positions.get(1));
        servoBackRight.setPosition(positions.get(2));
        servoFrontRight.setPosition(positions.get(3));
    }

    public void update() {
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontRight.setPower(frontRightPower);

        switch (state) {
            case MECANUM:
                servoFrontLeft.setPosition(FRONT_LEFT_IN);
                servoBackLeft.setPosition(BACK_LEFT_IN);
                servoBackRight.setPosition(BACK_RIGHT_IN);
                servoFrontRight.setPosition(FRONT_RIGHT_IN);
                break;
            case TRACTION:
                servoFrontLeft.setPosition(FRONT_LEFT_OUT);
                servoBackLeft.setPosition(BACK_LEFT_OUT);
                servoBackRight.setPosition(BACK_RIGHT_OUT);
                servoFrontRight.setPosition(FRONT_RIGHT_OUT);
                break;
            case STANDSTILL:
                servoFrontLeft.setPosition(FRONT_LEFT_SS);
                servoBackLeft.setPosition(BACK_LEFT_SS);
                servoBackRight.setPosition(BACK_RIGHT_SS);
                servoFrontRight.setPosition(FRONT_RIGHT_SS);
                break;
        }
    }

    public enum State {
        MECANUM,
        TRACTION,
        STANDSTILL,
    }


    /**
     * Interface that takes the two main methods of {@link Drive}: {@code setDriveSignal()}
     * and {@code setDrivePower()}. This removes the overhead of localization and such.
     */
    public interface SimpleDrive {
        /**
         * Sets the current commanded drive state of the robot and returns the corresponding
         * motor powers. Feedforward is applied to {@code driveSignal} before it reaches
         * the motors.
         *
         * @param driveSignal The desired velocity and acceleration.
         * @return The corresponding motor powers.
         */
        List<Double> setDriveSignal(DriveSignal driveSignal);

        /**
         * Sets the current commanded drive state of the robot and returns the corresponding
         * motor powers. Feedforward is *not* applied to {@code drivePower}.
         *
         * @param drivePower The desired velocity.
         * @return The corresponding motor powers.
         */
        List<Double> setDrivePower(Pose2d drivePower);
    }

    /**
     * Simple class that calculates the motor powers for a mecanum drive. Calculations are taken
     * from {@link MecanumDrive}. This is just a barebones class without the overhead of
     * localization and such.
     */
    public static class SimpleMecanumDrive implements SimpleDrive {
        private final double kV;
        private final double kA;
        private final double kStatic;
        private final double trackWidth;
        private final double wheelBase;
        private final double lateralMultiplier;

        /**
         * @param kV velocity feedforward
         * @param kA acceleration feedforward
         * @param kStatic additive constant feedforward
         * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
         * @param wheelBase distance between pairs of wheels on the same side of the robot
         * @param lateralMultiplier lateral multiplier
         */
        public SimpleMecanumDrive(
                double kV,
                double kA,
                double kStatic,
                double trackWidth,
                double wheelBase,
                double lateralMultiplier
        ) {
            this.kV = kV;
            this.kA = kA;
            this.kStatic = kStatic;
            this.trackWidth = trackWidth;
            this.wheelBase = wheelBase;
            this.lateralMultiplier = lateralMultiplier;
        }

        /**
         * @return A list of motor powers, with the four elements corresponding to the front left,
         * back left, back right, and front right powers, respectively.
         */
        @Override
        public List<Double> setDriveSignal(DriveSignal driveSignal) {
            List<Double> velocities = MecanumKinematics.robotToWheelVelocities(
                    driveSignal.getVel(),
                    trackWidth,
                    wheelBase,
                    lateralMultiplier
            );
            List<Double> accelerations = MecanumKinematics.robotToWheelAccelerations(
                    driveSignal.getAccel(),
                    trackWidth,
                    wheelBase,
                    lateralMultiplier
            );
            return Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        }

        /**
         * @return A list of motor powers, with the four elements corresponding to the front left,
         * back left, back right, and front right powers, respectively.
         */
        @Override
        public List<Double> setDrivePower(Pose2d drivePower) {
            return MecanumKinematics.robotToWheelVelocities(drivePower, trackWidth, wheelBase, lateralMultiplier);
        }
    }

    /**
     * Simple class that calculates the motor powers for a tank drive. Calculations are taken
     * from {@link TankDrive}. This is just a barebones class without the overhead of
     * localization and such.
     */
    public static class SimpleTankDrive implements SimpleDrive {
        private final double kV;
        private final double kA;
        private final double kStatic;
        private final double trackWidth;

        /**
         * @param kV velocity feedforward
         * @param kA acceleration feedforward
         * @param kStatic additive constant feedforward
         * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
         */
        public SimpleTankDrive(double kV, double kA, double kStatic, double trackWidth) {
            this.kV = kV;
            this.kA = kA;
            this.kStatic = kStatic;
            this.trackWidth = trackWidth;
        }

        /**
         * @return A list of motor powers, with the two elements corresponding to the left side
         * and the right side powers, respectively.
         */
        @Override
        public List<Double> setDriveSignal(DriveSignal driveSignal) {
            List<Double> velocities = TankKinematics.robotToWheelVelocities(driveSignal.getVel(), trackWidth);
            List<Double> accelerations = TankKinematics.robotToWheelAccelerations(driveSignal.getAccel(), trackWidth);
            return Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        }

        /**
         * @return A list of motor powers, with the two elements corresponding to the left side
         * and the right side powers, respectively.
         */
        @Override
        public List<Double> setDrivePower(Pose2d drivePower) {
            return TankKinematics.robotToWheelVelocities(drivePower, trackWidth);
        }
    }
}
