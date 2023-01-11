package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
 * slot's motor direction
 */
public class Encoder {
    private final static int CPS_STEP = 0x10000;

    private static double inverseOverflow(double input, double estimate) {
        // convert to uint16
        int real = (int) input & 0xffff;
        // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
        // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
        real += ((real % 20) / 4) * CPS_STEP;
        // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
        real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
        return real;
    }

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private DcMotorEx motor;
    private Direction direction;

    private int lastPosition = 0;
    private double rawVelocity = 0;
    private int velocityEstimateIndex = 0;
    private final double[] velocityEstimates = new double[3];
    private double lastUpdateTime;

    public Encoder(DcMotorEx motor, Direction direction) {
        this.motor = motor;
        this.direction = direction;
        this.lastUpdateTime = System.nanoTime();
    }

    public Encoder(DcMotorEx motor) {
        this(motor, Direction.FORWARD);
    }

    public Direction getDirection() {
        return direction;
    }

    private int getMultiplier() {
        return getDirection().getMultiplier() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Gets the position from the underlying motor and adjusts for the set direction.
     * Additionally, this method updates the velocity estimates used for compensated velocity
     *
     * @return encoder position
     */
    public int getCurrentPosition() {
        return lastPosition;
    }

    /**
     * Gets the velocity directly from the underlying motor and compensates for the direction
     * See {@link #getCorrectedVelocity} for high (>2^15) counts per second velocities (such as on REV Through Bore)
     *
     * @return raw velocity
     */
    public double getRawVelocity() {
        return rawVelocity;
    }

    /**
     * Uses velocity estimates gathered in {@link #getCurrentPosition} to estimate the upper bits of velocity
     * that are lost in overflow due to velocity being transmitted as 16 bits.
     * CAVEAT: must regularly call {@link #getCurrentPosition} for the compensation to work correctly.
     *
     * @return corrected velocity
     */
    public double getCorrectedVelocity() {
        double median = velocityEstimates[0] > velocityEstimates[1]
                ? Math.max(velocityEstimates[1], Math.min(velocityEstimates[0], velocityEstimates[2]))
                : Math.max(velocityEstimates[0], Math.min(velocityEstimates[1], velocityEstimates[2]));
        return inverseOverflow(getRawVelocity(), median);
    }

    public void update() {
        rawVelocity = motor.getVelocity() * getMultiplier();
        int position = motor.getCurrentPosition() * getMultiplier();
        double time = System.nanoTime() / 1e9;
        velocityEstimates[velocityEstimateIndex] = (position - lastPosition) / (time - lastUpdateTime);
        velocityEstimateIndex = (velocityEstimateIndex + 1) % 3;
        lastPosition = position;
        lastUpdateTime = time;
    }
}
