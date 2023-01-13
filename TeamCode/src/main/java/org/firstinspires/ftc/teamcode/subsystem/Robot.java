package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.opmode.test.HardwareTest.servoCurrent;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystem.io.IntakeOuttake;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class Robot {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public ButterflyRR butterfly;
    public IntakeOuttake intakeOuttake;
    public Odometry odometry;
    public BNO055IMU imu;
    public Webcam webcam;
    public List<LynxModule> lynxModules;
    public FtcDashboard dashboard;

    private Orientation orientation = new Orientation();
    private Pose2d position = new Pose2d();  // origin is back left of the field?  x is forward, y is right, z is up
    private Pose2d positionOffset = new Pose2d();
    private Pose2d velocity = new Pose2d();
    private double updateRate = 0;
    private long lastUpdateTime = System.nanoTime();

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        butterfly = new ButterflyRR(hardwareMap, this::getPosition, this::getVelocity);
        intakeOuttake = new IntakeOuttake(hardwareMap, this::getPosition);
        odometry = new Odometry(hardwareMap);  // todo

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new SimpleIMUIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // locally store the LynxModules since a synchronized block occurs each time we call hardwareMap.get(), so maybe saves a bit of time when we clear cache?
        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (DcMotor motor : new DcMotor[] { butterfly.frontLeft, butterfly.frontRight, butterfly.backLeft, butterfly.backRight,
                intakeOuttake.vertical.motors, intakeOuttake.turret.motor }) {
            MotorConfigurationType configuration = motor.getMotorType().clone();
            configuration.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(configuration);
        }

        dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        this.telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        this.telemetry.update();
        this.telemetry.clearAll();

        PhotonCore.enable();

//        update();
    }

    // todo perhaps multiple cameras and multiple pipelines. also add our own ml?
    public void initWebcam(OpenCvPipeline pipeline) {
        webcam = new Webcam(hardwareMap, pipeline);
    }

    /**
     * The robot's intrinsic orientation with axis order ZYX (heading, pitch, roll).
     * @return The robot's angular orientation.
     */
    public Orientation getOrientation() {
        return orientation;
    }

    /**
     * Left/right rotation in radians. From a top-down view of the robot, angle increases
     * in the clockwise direction. (AKA yaw.)
     * @return Radian angle measure of the robot's heading.
     */
    public double getHeading() {
        return orientation.firstAngle;
    }

    /**
     * Up/down rotation in radians. From the robot's perspective, angle increases as you
     * tilt downwards. (Although it's usually the z-axis, the robot's pitch is the y-axis
     * here.)
     * @return Radian angle measure of the robot's pitch.
     */
    public double getPitch() {
        return orientation.secondAngle;
    }

    /**
     * Side-to-side rotation in radians. From the robot's perspective, angle increases
     * as you tilt rightwards. (Although it's usually the y-axis, the robot's roll is
     * the z-axis here.)
     * @return Radian angle measure of the robot's roll.
     */
    public double getRoll() {
        return orientation.thirdAngle;
    }

    public Pose2d getPosition() {
        return position;
    }

    public void setPosition(Pose2d position) {
        positionOffset = position.minus(this.position);
    }

    public Pose2d getVelocity() {
        return velocity;
    }

    public double getUpdateRate() {
        return updateRate;
    }


    /**
     * Clear the bulk cache for each {@link LynxModule} in the robot.
     */
    public void clearCache() {
        for (LynxModule module : lynxModules) {
            module.clearBulkCache();
        }
    }

    public void updateIMU() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public void updatePosition() {
        position = odometry.getPoseEstimate().plus(positionOffset);
        velocity = odometry.getPoseVelocity();
    }

    public void update() {
        long time = System.nanoTime();
        updateRate = 1E9 / (time - lastUpdateTime);
        lastUpdateTime = time;

        updateIMU();
        updatePosition();
        clearCache();
        butterfly.update();
        intakeOuttake.update();
        odometry.update();
        telemetry.update();
    }

    public void telemetryCurrent() throws InterruptedException {
        double butterflyCurrent = butterfly.frontLeft.getCurrent(CurrentUnit.AMPS) + butterfly.backLeft.getCurrent(CurrentUnit.AMPS)
                + butterfly.backRight.getCurrent(CurrentUnit.AMPS) + butterfly.frontRight.getCurrent(CurrentUnit.AMPS);
        telemetry.addData("Butterfly Motors Current (amps)", butterflyCurrent);
        for (LynxModule lynxModule : lynxModules) {
            telemetry.addData(String.format("<code>%s</code> \"%s\" Servo Current (amps)", lynxModule.getDeviceName(), hardwareMap.getNamesOf(lynxModule).iterator().next()), servoCurrent(lynxModule));
        }
        telemetry.addData("Vertical Motors Current (amps)", intakeOuttake.vertical.motors.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Turret Motor Current (amps)", intakeOuttake.turret.motor.getCurrent(CurrentUnit.AMPS));
    }

    public void telemetryTime() {
        telemetry.addData("Update Rate", updateRate);
    }

    @Nullable
    private static LynxModule getControlHub(List<LynxModule> lynxModules) {
        for (LynxModule lynxModule : lynxModules) {
            if (LynxConstants.isEmbeddedSerialNumber(lynxModule.getSerialNumber())) {
                return lynxModule;
            }
        }
        return null;
    }
}
