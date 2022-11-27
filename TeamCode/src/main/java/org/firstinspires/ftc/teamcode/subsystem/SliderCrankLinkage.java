package org.firstinspires.ftc.teamcode.subsystem;

import kotlin.NotImplementedError;

/**
 * <p>A slider-crank linkage is a type of linkage with one point of rotation which translates to
 * linear motion via a constrained slider. See <a href="https://en.wikipedia.org/wiki/Slider-crank_linkage">Wikipedia</a>
 * for more information. You can also see <a href="https://www.desmos.com/calculator/skbfvczwpb">this interactive graph</a>
 * which calculates some useful information of the slider-crank linkage.</p>
 *
 * <p>In here, we call the axis the line where the linear motion is constrained. The base is the point on the axis that is
 * closest to the point of rotation of the crank. The slider is the point where the connecting rod connects to the axis.
 * We assume that the axis runs horizontally, with positive values going to the right. The crank is offset vertically, with
 * positive values going downwards. The angle of the crank is 0 when the crank is parallel to the axis and pointing to the
 * right. The angle increases as the crank rotates counterclockwise, using the above assumptions.</p>
 */
public class SliderCrankLinkage {

    public double r;
    public double l;
    public double o;

    /**
     * Models an offset slider-crank linkage with the given values. Ensure that all the values use consistent units.
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param crank The length of the crank arm.
     * @param rod The length of the connecting rod.
     * @param offset How far the crank's point of rotation is below the axis
     */
    public SliderCrankLinkage(double crank, double rod, double offset) {
        this.r = crank;
        this.l = rod;
        this.o = offset;
    }

    /**
     * Models an in-line slider-crank linkage with the given values. Ensure that all the values use consistent units.
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param crank The length of the crank arm.
     * @param rod The length of the connecting rod.
     */
    public SliderCrankLinkage(double crank, double rod) {
        this(crank, rod, 0);
    }

    /**
     * The distance between the slider and the base at a given angle of the crank.
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param a The current angle of the crank, in radians.
     * @return Distance between the slider and the base.
     */
    public double position(double a) {
        return r * Math.cos(a) + Math.sqrt(sq(l) - sq(r * Math.sin(a) - o));
    }

    /**
     * The velocity of the slider at a given angle of the crank with the specified angular velocity.
     * This is just the derivative of the {@link #position(double)} and doesn't take into account
     * other factors like torque or mass.
     *
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param a The current angle of the crank.
     * @param angleVelo The current angular velocity of the crank.
     * @return Velocity of the slider.
     */
    public double velocity(double a, double angleVelo) {
        double x = r * Math.sin(a) + o;
        double rawVelo = -r * Math.cos(a) * x / Math.sqrt(sq(l) - sq(x)) - (x - o);
        return rawVelo * angleVelo;
    }

    /**
     * The acceleration of the slider at a given angle of the crank with the specified angular
     * acceleration. This is just the second derivative of the {@link #position(double)} and doesn't
     * take into account other factors like torque or mass.
     *
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param a The current angle of the crank.
     * @param angleAccel The current angular acceleration of the crank.
     * @return Acceleration of the slider.
     */
    public double acceleration(double a, double angleAccel) {
        double x = r * Math.sin(a) + o;
        double y = r * Math.cos(a);
        double rawAccel = ((x - o) * x - sq(y)) / Math.sqrt(sq(l) - sq(x))
                - sq(y * x) / Math.pow(sq(l) - sq(x), 1.5) - y;
        return rawAccel * angleAccel;
    }

    /**
     * The force on the slider at a given angle of the crank with the specified torque.
     *
     * <p>NOTE: This only works for in-line slider-crank linkages, and it is also only a rough
     * approximation.</p>
     *
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param a The current angle of the crank.
     * @param torque THe current torque on the crank.
     * @return Force on the slider.
     */
    public double force(double a, double torque) {
        return torque / (r * Math.sin(a + Math.asin(r * Math.sin(a) / l)));
    }

    //todo the inverses of all of these
    //ie calculate crank angle given slider position.

    public double positionInv(double d) {
        throw new NotImplementedError();
    }

    public double velocityInv(double d, double velocity) {
        throw new NotImplementedError();
    }

    public double accelerationInv(double d, double acceleration) {
        throw new NotImplementedError();
    }

    public double torque(double d, double force) {
        throw new NotImplementedError();
    }

    private static double sq(double x) {
        return x*x;
    }
}
