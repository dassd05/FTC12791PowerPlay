package org.firstinspires.ftc.teamcode.util;

import java.util.concurrent.TimeUnit;

public class Timer {
    public static long time(Runnable action) {
        long time = System.nanoTime();
        action.run();
        return System.nanoTime() - time;
    }

    public static long time(Runnable action, TimeUnit timeUnit) {
        return timeUnit.convert(time(action), TimeUnit.NANOSECONDS);
    }

    public static long time(Runnable action, int times) {
        long time = System.nanoTime();
        for (int i = 0; i < times; i ++) action.run();
        return (System.nanoTime() - time) / times;
    }

    public static long time(Runnable action, int times, TimeUnit timeUnit) {
        return timeUnit.convert(time(action, times), TimeUnit.NANOSECONDS);
    }
}
