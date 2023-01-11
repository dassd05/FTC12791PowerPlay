package org.firstinspires.ftc.teamcode.opmode.test;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.view.View;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayDeque;
import java.util.Deque;

@Config
@TeleOp(group = "test")
public class RevSensorsTest extends LinearOpMode {

    public View relativeLayout;

    public RevColorSensorV3 color1, color2;
    public Rev2mDistanceSensor distance1, distance2;

    public ElapsedTime ledTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//    public Deque<Long> frames;

    public static double gain1 = 4;
    public static double gain2 = 4;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
//        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
//        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        distance1 = hardwareMap.get(Rev2mDistanceSensor.class, "distance1");
//        distance2 = hardwareMap.get(Rev2mDistanceSensor.class, "distance2");

        waitForStart();

//        frames = new ArrayDeque<>(100);
//        for (int i = 0; i < 99; i ++) {
//            frames.add(System.nanoTime());
//        }

        while (opModeIsActive()) {
            color1.setGain((float)gain1);
//            color2.setGain((float)gain2);
            NormalizedRGBA colors1 = color1.getNormalizedColors();
//            NormalizedRGBA colors2 = color2.getNormalizedColors();

            // doesnt seem to be working
            // Change the Robot Controller's background color to match the color detected by the color sensor.
//            relativeLayout.post(() -> relativeLayout.setBackgroundColor(colors1.toColor()));

//            if (ledTimer.time() > 10000) {
//                // doesnt work for rev color sensors because they are broadcom
//                // idk why
//                color1.enableLed(!color1.isLightOn());
//                ledTimer.reset();
//            }

//            frames.add(System.nanoTime());
//            telemetry.addData("fps", 1E11 / (double)(frames.getLast() - frames.remove()));
            telemetry.addData("1 distance (mm)", color1.getDistance(DistanceUnit.MM));
            telemetry.addData("1 color (RGBA)", String.format("%3.1f %3.1f %3.1f %3.1f", colors1.red * 256, colors1.green * 256, colors1.blue * 256, colors1.alpha * 256));
            telemetry.addData("1 light detected", color1.getLightDetected());
//            telemetry.addData("2 distance", color2.getDistance(DistanceUnit.INCH) + " in");
//            telemetry.addData("2 color (RGBA)", String.format("%3.1f %3.1f %3.1f %3.1f", colors2.red * 256, colors2.green * 256, colors2.blue * 256, colors2.alpha * 256));
//            telemetry.addData("2 light detected", color2.getLightDetected());

            telemetry.addData("distance1 (mm)", distance1.getDistance(DistanceUnit.MM));
//            telemetry.addData("distance2", String.format("%2fcm", distance2.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}
