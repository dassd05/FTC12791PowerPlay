package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.vision.JunctionDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.vision.SignalDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

public class Robot {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Butterfly butterfly;
    public Deployment deployment;
    public BNO055IMU imu;
    public Webcam webcam;
    public FtcDashboard dashboard;

    public Orientation orientation;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        butterfly = new Butterfly(hardwareMap);
        deployment = new Deployment(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (DcMotorEx motor : new DcMotorEx[] { butterfly.frontLeft, butterfly.frontRight, butterfly.backLeft, butterfly.backRight, deployment.vertical }) {
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

    public void initAuton() {
        webcam = new Webcam(hardwareMap, new SignalDetectionPipeline());
    }

    public void initTeleop() {
        webcam = new Webcam(hardwareMap, new JunctionDetectionPipeline());
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
        orientation = imu.getAngularOrientation();
        clearCache();
        butterfly.update();
        deployment.update();
        telemetry.update();
    }
}
