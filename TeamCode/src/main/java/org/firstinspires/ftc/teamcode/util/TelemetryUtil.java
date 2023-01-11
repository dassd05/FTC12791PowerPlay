package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUtil {
    public static Telemetry initTelemetry(Telemetry telemetry) {
        Telemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        multiTelemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        multiTelemetry.update();
        multiTelemetry.clearAll();
        return multiTelemetry;
    }
}
