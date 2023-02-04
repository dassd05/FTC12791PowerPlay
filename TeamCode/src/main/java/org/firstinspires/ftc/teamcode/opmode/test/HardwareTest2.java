package org.firstinspires.ftc.teamcode.opmode.test;

import android.content.Context;
import android.os.CpuUsageInfo;
import android.os.HardwarePropertiesManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@Disabled
@TeleOp(group = "test")
public class HardwareTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Context context = AppUtil.getInstance().getApplication();
        HardwarePropertiesManager hardwareProperties = (HardwarePropertiesManager) context.getSystemService(Context.HARDWARE_PROPERTIES_SERVICE);

        waitForStart();

        while (opModeIsActive()) {
            CpuUsageInfo[] cpuUsageInfo = hardwareProperties.getCpuUsages();
            float[] fanSpeeds = hardwareProperties.getFanSpeeds();
            float[] cpuTemps = hardwareProperties.getDeviceTemperatures(HardwarePropertiesManager.DEVICE_TEMPERATURE_CPU, HardwarePropertiesManager.TEMPERATURE_CURRENT);
            float[] gpuTemps = hardwareProperties.getDeviceTemperatures(HardwarePropertiesManager.DEVICE_TEMPERATURE_GPU, HardwarePropertiesManager.TEMPERATURE_CURRENT);
            float[] batteryTemps = hardwareProperties.getDeviceTemperatures(HardwarePropertiesManager.DEVICE_TEMPERATURE_BATTERY, HardwarePropertiesManager.TEMPERATURE_CURRENT);
            float[] skinTemps = hardwareProperties.getDeviceTemperatures(HardwarePropertiesManager.DEVICE_TEMPERATURE_SKIN, HardwarePropertiesManager.TEMPERATURE_CURRENT);

            for (int i = 0; i < cpuUsageInfo.length; i ++) telemetry.addData("CPU " + i + " Usage Time Percent", 100d * cpuUsageInfo[i].getActive() / cpuUsageInfo[i].getTotal());
            for (int i = 0; i < fanSpeeds.length; i ++) telemetry.addData("Fan " + i + " Speed", fanSpeeds[i]);
            for (int i = 0; i < cpuTemps.length; i ++) telemetry.addData("CPU " + i + " Temperature", cpuTemps[i]);
            for (int i = 0; i < gpuTemps.length; i ++) telemetry.addData("GPU " + i + " Temperature", gpuTemps[i]);
            for (int i = 0; i < batteryTemps.length; i ++) telemetry.addData("Battery " + i + " Temperature", batteryTemps[i]);
            for (int i = 0; i < skinTemps.length; i ++) telemetry.addData("Skin " + i + " Temperature", skinTemps[i]);
            telemetry.update();
        }
    }
}
