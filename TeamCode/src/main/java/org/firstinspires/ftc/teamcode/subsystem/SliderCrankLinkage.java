package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.teamcode.util.GraphFunction;

import java.util.List;

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

    public double crank;
    public double rod;
    public double offset;
    private GraphFunction positionCache;
    private GraphFunction velocityCache;
    private GraphFunction accelerationCache;
    private GraphFunction torqueCache;

    /**
     * Models an offset slider-crank linkage with the given values. Ensure that all the values use consistent units.
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param crank The length of the crank arm.
     * @param rod The length of the connecting rod.
     * @param offset How far the crank's point of rotation is below the axis
     */
    public SliderCrankLinkage(double crank, double rod, double offset) {
        this.crank = crank;
        this.rod = rod;
        this.offset = offset;
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
        return crank * Math.cos(a) + Math.sqrt(sq(rod) - sq(crank * Math.sin(a) - offset));
    }

    /**
     * The velocity of the slider at a given angle of the crank with the specified angular velocity.
     * This is just the derivative of the {@link #position(double)} and doesn't take into account
     * other factors like torque or mass.
     *
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param a The current angle of the crank, in radians.
     * @param angleVelo The current angular velocity of the crank, in radians/second.
     * @return Velocity of the slider.
     */
    public double velocity(double a, double angleVelo) {
        double x = crank * Math.sin(a) + offset;
        double rawVelo = -crank * Math.cos(a) * x / Math.sqrt(sq(rod) - sq(x)) - (x - offset);
        return rawVelo * angleVelo;
    }

    /**
     * The acceleration of the slider at a given angle of the crank with the specified angular
     * acceleration. This is just the second derivative of the {@link #position(double)} and doesn't
     * take into account other factors like torque or mass.
     *
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param a The current angle of the crank, in radians.
     * @param angleAccel The current angular acceleration of the crank, in radians/second<sup>2</sup>.
     * @return Acceleration of the slider.
     */
    public double acceleration(double a, double angleAccel) {
        double x = crank * Math.sin(a) + offset;
        double y = crank * Math.cos(a);
        double rawAccel = ((x - offset) * x - sq(y)) / Math.sqrt(sq(rod) - sq(x))
                - sq(y * x) / Math.pow(sq(rod) - sq(x), 1.5) - y;
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
     * @param a The current angle of the crank, radians.
     * @param torque The current torque on the crank.
     * @return Force on the slider.
     */
    public double force(double a, double torque) {
        return torque / (crank * Math.sin(a + Math.asin(crank * Math.sin(a) / rod)));
    }

    //todox the inverses of all of these
    //ie calculate crank angle given slider position.
    // the inversion functions are complicated as fuck :/
    // also there'll be two valid angles per position, so how tf do we calculate??

    public void calculateInverses(double refinement) {
        positionCache = new GraphFunction(this::position, 0, 2*Math.PI, refinement);
        velocityCache = positionCache.derivative();
        accelerationCache = velocityCache.derivative();
        torqueCache = new GraphFunction((Double x) -> force(x, 1), 0, 2*Math.PI, refinement);
    }

    /**
     * The angle of the crank given at a distance between the slider and the base.
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param d The current distance between the slider and the base.
     * @return Angle of the crank.
     */
    public List<Double> positionInv(double d) {
        return positionCache.findAllX(d);
    }

    /**
     * The angular velocity of the crank at a given distance between the slider and the base with
     * the specified slider velocity. This is just the derivative of the
     * {@link #positionInv(double)} and doesn't take into account other factors like torque or mass.
     *
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param d The current distance between the slider and the base.
     * @param velocity The current velocity of the slider.
     * @return Angular velocity of the crank, in radians/second.
     */
    public List<Double> velocityInv(double d, double velocity) {
        List<Double> velocities = velocityCache.findAllX(d);
        velocities.replaceAll((Double x) -> x * velocity);
        return velocities;
    }

    /**
     * The angular acceleration of the crank at a given distance between the slider and the base
     * with the given slider acceleration. This is just the second derivative of the
     * {@link #position(double)} and doesn't take into account other factors like torque or mass.
     *
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param d The current distance between the slider and the base.
     * @param acceleration The current acceleration of the slider
     * @return Angular acceleration of the crank, radians/second<sup>2</sup>.
     */
    public List<Double> accelerationInv(double d, double acceleration) {
        List<Double> accelerations = accelerationCache.findAllX(d);
        accelerations.replaceAll((Double x) -> x * acceleration);
        return accelerations;
    }

    /**
     * The torque on the crank at a given distance between the slider and the base with the
     * specified force on the slider.
     *
     * <p>NOTE: This only works for in-line slider-crank linkages, and it is also only a rough
     * approximation.</p>
     *
     * <p>*Make sure you read the assumptions outlined in {@link SliderCrankLinkage}!*</p>
     *
     * @param d The current distance between the slider and the base.
     * @param force The current force on the slider.
     * @return Torque on the crank.
     */
    public List<Double> torque(double d, double force) {
        List<Double> torques = torqueCache.findAllX(d);
        torques.replaceAll((Double x) -> x * force);
        return torques;
    }

    private static double sq(double n) {
        return n*n;
    }
}
