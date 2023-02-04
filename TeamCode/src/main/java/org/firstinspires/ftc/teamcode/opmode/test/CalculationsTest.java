package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.SliderCrankLinkage;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(group = "test")
public class CalculationsTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);

        double x = Math.random();
        int n = 1000000;
        Runnable calculation;
        calculation = () -> {
            double pi = 1;
            for (int i = 1; i <= n; i ++) {
                pi *= ((i+1) >> 1 << 1);
                pi /= ((i >> 1 << 1) + 1);
            }
            pi *= 2;
        };  // wallis product
        calculation = () -> {
            double e = 0;
            for (int i = 0; i < n; i ++) {
                double factorial = 1;
                for (int j = 1; j <= i; j ++) factorial *= j;
                e += Math.pow(x, i) / factorial;
            }
        };  // exp(x) = e^x
        calculation = () -> {
            List<Integer> primes = new ArrayList<>(n >> 3);
            primes.add(2);
            for (int i = 3; i <= n; i += 2) {
                int sqrt = (int) Math.sqrt(i);
                for (int prime : primes) {
                    if (i % prime == 0) break;
                    if (prime > sqrt) {
                        primes.add(i);
                        break;
                    }
                }
            }
        };  // primes
        calculation = () -> {
            double f = x / Math.PI;
            double sin = 1;
            for (int i = 2; i < n+2; i ++) {
//                int k = (i >> 1 << 1 == i) ? i >> 1 : -(i >> 1);
//                sin *= 1 - f/k;
                sin *= 1 - ((i % 2 << 1) - 1) * f / (i >> 1);
            }
            sin *= x;
        };  // sin(x)
        calculation = () -> {
            double sqrt = 1;
            for (int i = 0; i < n; i ++) {
//                double diff = x - sqrt*sqrt;
//                sqrt += .5 * diff / sqrt;
                sqrt += .5 * (x / sqrt - sqrt);
                /*
                x = n^2 + d
                d -> 0
                n = ?
                n * (1 - d / 2n^2) = n - d / 2n
                x = n^2 - d + d^2/4n^2 + d = n^2 + d^2/4n^2
                wow this algorithm i just made up converges extremely fast given a decent initial guess for n
                within ten iterations you can't get more accurate than a floating point error!
                decent = within an order of magnitude or two
                converges in roughly n iterations, where n is the orders of two apart the initial guess and final answer are.
                 */
            }
        };  // surprisingly decent implementation of sqrt
        calculation = () -> {
            SliderCrankLinkage linkage = new SliderCrankLinkage(x * 10, Math.sqrt(x) * 10);
            linkage.calculateInverses(2*Math.PI / n);
        };

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Calculation Time", Timer.time(calculation, TimeUnit.MILLISECONDS) + " ms");
            telemetry.update();
        }
    }
}
