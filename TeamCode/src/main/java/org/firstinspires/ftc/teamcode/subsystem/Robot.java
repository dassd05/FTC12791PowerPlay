package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystem.io.IntakeOuttake;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.easyopencv.OpenCvPipeline;

public class Robot {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Butterfly butterfly;
    public IntakeOuttake intakeOuttake;
    public BNO055IMU imu;
    public Webcam webcam;
    public FtcDashboard dashboard;

    public Orientation orientation;
    public Pose2d position;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        butterfly = new Butterfly(hardwareMap, () -> position);
        intakeOuttake = new IntakeOuttake(hardwareMap, () -> position);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new SimpleIMUIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
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
    }

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


    /**
     * Clear the bulk cache for each {@link LynxModule} in the robot.
     */
    public void clearCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
    }

    public void update() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        clearCache();
        butterfly.update();
        intakeOuttake.update();
        telemetry.update();
    }
}
