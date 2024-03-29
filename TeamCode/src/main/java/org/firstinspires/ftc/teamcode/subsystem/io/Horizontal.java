package org.firstinspires.ftc.teamcode.subsystem.io;

import static org.firstinspires.ftc.teamcode.opmode.test.HardwareTest.servoCurrent;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.ServoStuff;
import org.firstinspires.ftc.teamcode.subsystem.SliderCrankLinkage;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Config
public class Horizontal {
    public static final double MAX_FORWARD = 2 * 360;
    public static final double MAX_BACKWARD = 2 * 240;

    //rad
    public static double safetyClipMin = .141592653589;
    public static double safetyClipMax = 3.0;

    public static boolean debugging = false;

    public HardwareMap hardwareMap;
    public Servo forwardLeft;
    public Servo forwardRight;
    public Servo backwardLeft;
    public Servo backwardRight;

    public SliderCrankLinkage forward;
    public SliderCrankLinkage backward;

    private double position = 0;
    private double target = 0;
    private double forwardTarget = 0;
    private double backwardTarget = 0;

    public Horizontal(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        forwardLeft = hardwareMap.get(Servo.class, "hfl");
        forwardRight = hardwareMap.get(Servo.class, "hfr");
        backwardLeft = hardwareMap.get(Servo.class, "hbl");
        backwardRight = hardwareMap.get(Servo.class, "hbr");

//        forwardLeft.setPwmRange(ServoStuff.AxonMiniServo.servoModePwmRange);
//        forwardRight.setPwmRange(ServoStuff.AxonMiniServo.servoModePwmRange);
//        backwardLeft.setPwmRange(ServoStuff.AxonMaxServo.servoModePwmRange);
//        backwardRight.setPwmRange(ServoStuff.AxonMaxServo.servoModePwmRange);

        // positive position = outwards
//        forward1.setDirection(Servo.Direction.REVERSE);  // no use for this since we use Range.scale from angle to position
//        backward1.setDirection(Servo.Direction.REVERSE);

        forward = new SliderCrankLinkage(360, 383.228);
        backward = new SliderCrankLinkage(240, 277.684);
        forward.calculateInverses(.001);  // ~6000 (.001 in 2pi) * 4 (p, v, a, t) * 2 (f + b) = 48000 (hard - p, v, a, t) calculations
        backward.calculateInverses(.001);  // 4800 * 8 (bytes per double) * 2 (x + y) = 384000 bytes

        // should we constrain the velocity of the linkage with a ServoEx class?
    }

    // mm
    public double getPosition() {
        double fla = Range.scale(forwardLeft.getPosition(), Constants.DEPLOYMENT.FORWARD_LEFT_IN, Constants.DEPLOYMENT.FORWARD_LEFT_OUT, 0, Math.PI);
        double fra = Range.scale(forwardRight.getPosition(), Constants.DEPLOYMENT.FORWARD_RIGHT_IN, Constants.DEPLOYMENT.FORWARD_RIGHT_OUT, 0, Math.PI);
        double bla = Range.scale(backwardLeft.getPosition(), Constants.DEPLOYMENT.BACKWARD_LEFT_IN, Constants.DEPLOYMENT.BACKWARD_LEFT_OUT, 0, Math.PI);
        double bra = Range.scale(backwardRight.getPosition(), Constants.DEPLOYMENT.BACKWARD_RIGHT_IN, Constants.DEPLOYMENT.BACKWARD_RIGHT_OUT, 0, Math.PI);
        return (forward.position(fla) + forward.position(fra) - backward.position(bla) - backward.position(bra)) * .5;
    }
    public double getTarget() {
        return target;
    }
    public void setTarget(double target) {
        this.target = Range.clip(target, -MAX_BACKWARD, MAX_FORWARD);
    }

    public void update() {
        // we just say that only one side is nonzero at any moment
        forwardTarget = target > 0 ? Math.PI - forward.positionInv(target + forward.rod - forward.crank).get(0) : 0; // if there are multiple values of the inverse, then we grab the lowest one
        backwardTarget = target < 0 ? Math.PI - backward.positionInv(-target + backward.rod - backward.crank).get(0) : 0; // thus it'll most likely be in the range [0, pi] todo

        // limit range so no hitting or weird behavior
        forwardTarget = Range.clip(forwardTarget, safetyClipMin, safetyClipMax);
        backwardTarget = Range.clip(backwardTarget, safetyClipMin, safetyClipMax);

//        forward1.setPosition(angleToAxonServo(forwardTarget) + FORWARD1_OFFSET);
//        forward2.setPosition(angleToAxonServo(forwardTarget) + FORWARD2_OFFSET);
//        backward1.setPosition(angleToAxonServo(backwardTarget) + BACKWARD1_OFFSET);
//        backward2.setPosition(angleToAxonServo(backwardTarget) + BACKWARD2_OFFSET);
        // ensure that we *NEVER* go past [0, pi]
        if (!debugging) {
            forwardLeft.setPosition(clipScale(forwardTarget, 0, Math.PI, Constants.DEPLOYMENT.FORWARD_LEFT_IN, Constants.DEPLOYMENT.FORWARD_LEFT_OUT));
            forwardRight.setPosition(clipScale(forwardTarget, 0, Math.PI, Constants.DEPLOYMENT.FORWARD_RIGHT_IN, Constants.DEPLOYMENT.FORWARD_RIGHT_OUT));
            backwardLeft.setPosition(clipScale(backwardTarget, 0, Math.PI, Constants.DEPLOYMENT.BACKWARD_LEFT_IN, Constants.DEPLOYMENT.BACKWARD_LEFT_OUT));
            backwardRight.setPosition(clipScale(backwardTarget, 0, Math.PI, Constants.DEPLOYMENT.BACKWARD_RIGHT_IN, Constants.DEPLOYMENT.BACKWARD_RIGHT_OUT));
        }
    }

    // cannot rely on angle to position conversions, since the servos apparently don't have the range they report they do
    //rad
    public static double angleToGobildaServo(double angle) {
        return Math.toDegrees(angle) / 300;
    }
    public static double angleToAxonServo(double angle) {
        return Math.toDegrees(angle) / 355;
    }

    private static double clipScale(double n, double a1, double a2, double b1, double b2) {
        // scale then clip in the SLIGHTEST chance of a floating point error and it extends outside the given range
        return clip(Range.scale(n, a1, a2, b1, b2), b1, b2);
    }
    private static double clip(double n, double a, double b) {
        if (a < b) return Math.min(Math.max(n, a), b);
        else return Math.min(Math.max(n, b), a);
    }


    @Config
    @TeleOp(group = "test")
    public static class HorizontalTest extends LinearOpMode {
        public static double target = 0;

        @Override
        public void runOpMode() throws InterruptedException {
            Horizontal horizontal = new Horizontal(hardwareMap);
            debugging = true;
            telemetry = TelemetryUtil.initTelemetry(telemetry);

            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            waitForStart();

            while (opModeIsActive()) {
                sleep((long) Math.max(0, 10 - timer.time()));
                timer.reset();

                target += 2 * gamepad1.right_stick_x;  // max 200 mm/s
                horizontal.setTarget(target);

                double current = 0;
                for (LynxModule lynxModule : hardwareMap.getAll(LynxModule.class))
                    current += servoCurrent(lynxModule);

                telemetry.addData("Horizontal Target", "%f mm", horizontal.getTarget());
                telemetry.addData("Horizontal Position", "%f mm", horizontal.getPosition());
                telemetry.addData("Forward Target", "%f°", Math.toDegrees(horizontal.forwardTarget));
                telemetry.addData("Backward Target", "%f°", Math.toDegrees(horizontal.backwardTarget));
                telemetry.addData("Forward Left Position", horizontal.forwardLeft.getPosition());
                telemetry.addData("Forward Right Position", horizontal.forwardRight.getPosition());
                telemetry.addData("Backward Left Position", horizontal.backwardLeft.getPosition());
                telemetry.addData("Backward Right Position", horizontal.backwardRight.getPosition());
                telemetry.addData("Servo current", current);
                telemetry.update();
                horizontal.update();
            }
        }
    }
}
