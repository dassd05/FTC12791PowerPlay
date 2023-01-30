package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {
    /*
    TODO Robot problems:
    ButterflyTest: gamepad driving is messed up (rightStickY controls forward/backward movement, leftStickY control rotation, and no strafing)
    HorizontalTest: still fucking messed up idk go graph it again, maybe check configuration/ports. ~~Solved, I think. Range.clip is really stupid where it always assumes that a < b.
    JustPressed: never initialized the hash set before and never had errors iirc, but now we have them. idk :shrug:
    TurretTest: when near full power, and suddenly robot crashed/hung, maybe a cable disconnect or smth. Yeah now turret motor doesnt seem to be working
    VerticalTest: draws WAY too much power. Also when coming all the way back down, it continues to draw several amps
    ArmTest: arm keeps on getting stuck on linkages and whatnot
    Horizontal: linkages get stuck on sides sometimes
    GraphFunction: not entirely sure if it's free from bugs. it's kinda jank.
    Odometry: don't really know exact position of the forward encoder. also positions will change between mecanum and tank drive on butterfly switching
    SimpleIMUIntegrator: haven't looked at in a bit, but just doesn't seem to want to work. Maybe was working last time, but horribly.
    ConeDetectionPipeline/JunctionDetectionPipeline: vision hard. idk

    **Find ticks to mm conversion for odometry encoders and vertical encoders**

    Overall: WAYYY too much friction in entire system. Feels like any hard movement will cause something to break dangerously.

    Delete random commented out code and random comments
    Write documentation
     */

    private Constants() {}

    public static final class BUTTERFLY {
        // maybe make a constant DISTANCE, and just do IN + DISTANCE for OUT.

        // here IN is when all 8 wheels are together (misleading ikr)
        // OUT is traction mode
        public static double FRONT_LEFT_IN = .29;
        public static double FRONT_LEFT_OUT = 0.; // less, but out of range
        public static double FRONT_RIGHT_IN = .61;
        public static double FRONT_RIGHT_OUT = .92;
        public static double BACK_LEFT_IN = .74;
        public static double BACK_LEFT_OUT = 1; // more, but out of range
        public static double BACK_RIGHT_IN = .49;
        public static double BACK_RIGHT_OUT = .18;


        public static double OUT_STAND_DISTANCE = .31; // distance between traction mode and all 8 wheels together
        public static double IN_STAND_DISTANCE = .2; // (arbitrary) distance between mecanum mode and all 8 wheels together
    }

    public static final class TURRET {

    }

    public static final class DEPLOYMENT {
        // long is forward is outtake, short is backward is intake
        public static double FORWARD_LEFT_IN = .765; // diff = .67
        public static double FORWARD_LEFT_OUT = .095;
        public static double FORWARD_RIGHT_IN = .085; // diff = .665
        public static double FORWARD_RIGHT_OUT = .75;
        public static double BACKWARD_LEFT_IN = .23; // diff = .555
        public static double BACKWARD_LEFT_OUT = .785;
        public static double BACKWARD_RIGHT_IN = .975; // diff = .565
        public static double BACKWARD_RIGHT_OUT = .425;
        public static double ARM_BACKWARD = .994; // diff = .266
        public static double ARM_FORWARD = .462; // diff = .266
        public static double ARM_UP = .728;

        // theoretically, axon has 355 degrees of freedom, so .507 difference should be 180 degrees
//        public static double FORWARD_180 = .666;
//        public static double BACKWARD_180 = .560;
//        public static double ARM_180 = .532;


        public static double v4bIntakePrep = .68;
        public static double v4bIntake = .78;
        public static double v4bNeutral = .4;
        public static double v4bOuttake = .25;

//        public static double intakeLinkage1In = .23;
//        public static double intakeLinkage2In = .975;
//        public static double intakeLinkage1Out = -1;
//        public static double getIntakeLinkage1Up = .51;
//        public static double intakeLinkage2Out = .43;
//
//        public static double outtakeLinkage1In = .09;
//        public static double outtakeLinkage2In = .76;

        public static double outtakeLinkage1Out = .55;
        public static double outtakeLinkage2Out = .3;

        public static double outtakeLinkage2SafeIntake = .6;
        public static double outtakeLinkage1SafeIntake = .25;


        public static double intakeLinkage1IntakePosTemp = .65;
        public static double intakeLinkage2IntakePosTenp = .56;

        public static double outtakeLinkage1IntakePosTemp = .4;
        public static double outakeLinkage2IntakePosTenp = .45;



        public static int slidesUp = 1650;
    }

    // todo should we have all constants centralized like these, or should each of these constants go into their respective class?

    public static final class PORTS {
        // motors are all GoBILDA 5202/3/4 series
        // except slides, which are Matrix 12v Motor
        public static final class CONTROL_HUB {
            public static final String[] MOTORS = new String[] {
                    "frontLeft",
                    "frontRight", // reverse
                    "backLeft",
                    "backRight" // reverse
            };
            public static final String[] SERVOS = new String[] {
                    "",
                    "rotation",
                    "backLeft",
                    "frontLeft",
                    "backRight",
                    "frontRight",
                    ""
            };
        }
        public static final class EXPANSION_HUB {
            public static final String[] MOTORS = new String[] {
                    "vertical",
                    "vertical",
                    "vertical",
                    "turret"
            };
            public static final String[] SERVOS = new String[] {
                    "claw",
                    "arm",
                    "backwardLeft",
                    "backwardRight",
                    "forwardRight",
                    "forwardLeft"
            };
        }
        public static final String[] SERVO_POWER_MODULE = new String[] {
                "backwardLeft",
                "",
                "forwardRight",
                "arm",
                "forwardLeft",
                ""
        };
    }
}
