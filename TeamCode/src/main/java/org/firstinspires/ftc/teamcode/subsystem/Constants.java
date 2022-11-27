package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {

    private Constants() {}

    public static final class BUTTERFLY {
        // maybe make a constant DISTANCE, and just do IN + DISTANCE for OUT.
        public static double FRONT_LEFT_IN = 0;
        public static double FRONT_LEFT_OUT = 0;
        public static double FRONT_RIGHT_IN = 0;
        public static double FRONT_RIGHT_OUT = 0;
        public static double BACK_LEFT_IN = 0;
        public static double BACK_LEFT_OUT = 0;
        public static double BACK_RIGHT_IN = 0;
        public static double BACK_RIGHT_OUT = 0;
        public static double IN_OUT_DISTANCE = 0;
        public static double IN_STILL_DISTANCE = 0;
    }

    public static final class TURRET {

    }

    public static final class DEPLOYMENT {

    }

    public static double BUTTERFLY_IN = 0;
    public static double BUTTERFLY_OUT = 0;
    public static double BUTTERFLY_STILL = 0;

    // todo should we have all constants centralized like these, or should each of these constants go into their respective class?
}
