package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class GraphFunction {
    private final Function<Double, Double> function;
    private double step;
    private final double xMin; // should we make the domain adjustable?
    private final double xMax;
    private double yMin;
    private double yMax;
    private double[] xAxis = null;
    private double[] yAxis = null;

    public GraphFunction(Function<Double, Double> function, double xMin, double xMax, double step) {
        this.function = function;
        this.xMin = xMin;
        this.xMax = xMax;
        recalculate(step);
    }

    public double getStep() {
        return step;
    }

    public double[] getDomain() {
        return new double[]{ xMin, xMax };
    }

    public double[] getRange() {
        return new double[]{ yMin, yMax };
    }

    public double[] getXAxis() {
        return xAxis;
    }

    public double[] getYAxis() {
        return yAxis;
    }

    public void recalculate(double step) {
        if (xMin > xMax) throw new IllegalArgumentException("xMin must be less than xMax.");
        if (step > xMax - xMin) throw new IllegalArgumentException("step too large given the x range.");

        this.step = step;

        int size = (int) Math.floor((xMax - xMin) / step + 1);
        xAxis = new double[size];
        yAxis = new double[size];
        double x = xMin;

        for (int i = 0; i < size; i++) {
            xAxis[i] = x;
            yAxis[i] = function.apply(x);
            x += step;

            if (i == 0) yMin = yMax = yAxis[i];
            yMin = Math.min(yMin, yAxis[i]);
            yMax = Math.max(yMax, yAxis[i]);
        }
    }

    private boolean inRoughDomain(double x) {
        return x >= xMin - step && x <= xMax + step;
    }
    private boolean inStrictDomain(double x) {
        return x >= xMin && x <= xMax;
    }
    private boolean inRoughRange(double y) {
        return y >= yMin - yStep() && y <= yMax + step;
    }
    private boolean inStrictRange(double y) {
        return y >= yMin && y <= yMax;
    }
    private double yStep() {
        return (yMax - yMin) / yAxis.length;
    }

    public double findY(double x) {
        if (xAxis == null || yAxis == null) throw new IllegalStateException("You need to calculate values first.");
        if (!inRoughDomain(x)) throw new IllegalArgumentException("x is outside of the defined domain.");

        double index = linearInterpolation(x, xMin, xMax, 0, xAxis.length-1);
        int i = (int) index; // should be equivalent to the Math.floor operation
        return linearInterpolation(index - i, yAxis[i], yAxis[i+1]);
    }

    //todo use the findAllX's algorithm and put into here and findLastX
    public double findFirstX(double y) {
        if (xAxis == null || yAxis == null) throw new IllegalStateException("You need to calculate values first.");
        if (!inRoughRange(y)) throw new IllegalArgumentException("y is outside of the defined range.");

        double prevY = yAxis[0];
        for (int i = 0; i < yAxis.length; i++) {
            if (yAxis[i] == y) return xAxis[i];
            if (i == 0) continue;

            if (prevY < y != yAxis[i] < y) {
                return linearInterpolation(y, prevY, yAxis[i], xAxis[i-1], xAxis[i]);
            }
        }
        throw new IllegalStateException("Somehow we were unable to find a corresponding x value.");
    }

    public double findLastX(double y) {
        if (xAxis == null || yAxis == null) throw new IllegalStateException("You need to calculate values first.");
        if (!inRoughRange(y)) throw new IllegalArgumentException("y is outside of the defined range.");

        double prevY = yAxis[yAxis.length-1];
        for (int i = yAxis.length-1; i >= 0; i--) {
            if (yAxis[i] == y) return xAxis[i];
            if (i == yAxis.length-1) continue;

            if (prevY < y != yAxis[i] < y) {
                return linearInterpolation(y, prevY, yAxis[i], xAxis[i+1], xAxis[i]);
            }
        }
        throw new IllegalStateException("Somehow we were unable to find a corresponding x value.");
    }

    public List<Double> findAllX(double y) {
        if (xAxis == null || yAxis == null) throw new IllegalStateException("You need to calculate values first.");
        if (!inRoughRange(y)) throw new IllegalArgumentException(String.format("y [%f] is outside of the defined range [%f, %f].", y, yMin, yMax));

        List<Double> solutions = new ArrayList<>();

        for (int i = 0; i < yAxis.length; i++) {
            if (inStrictRange(y)) {
                if (yAxis[i] == y) {
                    solutions.add(xAxis[i]);
                    continue;
                }
                if (i == 0 || yAxis[i-1] == y) continue;

                if (yAxis[i-1] < y != yAxis[i] < y) {
                    solutions.add(linearInterpolation(y, yAxis[i-1], yAxis[i], xAxis[i - 1], xAxis[i]));
                }
            } else {
                if (i == 0) {
                    if (y < yMin && yAxis[i] < yAxis[i+1] && y > yAxis[i] - yStep()) solutions.add(xAxis[i]);
                    if (y > yMax && yAxis[i] > yAxis[i+1] && y < yAxis[i] + yStep()) solutions.add(xAxis[i]);
                } else if (i == yAxis.length-1) {
                    if (y < yMin && yAxis[i] < yAxis[i-1] && y > yAxis[i] - yStep()) solutions.add(xAxis[i]);
                    if (y > yMax && yAxis[i] > yAxis[i-1] && y < yAxis[i] + yStep()) solutions.add(xAxis[i]);
                } else {
                    if (y < yMin && yAxis[i] < yAxis[i-1] && yAxis[i] < yAxis[i+1] && y > yAxis[i] - yStep()) solutions.add(xAxis[i]);
                    if (y > yMax && yAxis[i] > yAxis[i-1] && yAxis[i] > yAxis[i+1] && y < yAxis[i] + yStep()) solutions.add(xAxis[i]);
                }
            }
        }
        return solutions;
    }


    public GraphFunction derivative() {
        return new GraphFunction((Double x) -> (findY(x + step/2) - findY(x - step/2)) / step, xMin + step/2, xMax - step/2, step);
    }


    private static double linearInterpolation(double n, double a1, double a2) {
        return n * (a2 - a1) + a1;
    }
    private static double linearInterpolation(double n, double a1, double a2, double b1, double b2) {
        return (n - a1) / (a2 - a1) * (b2 - b1) + b1;
    }

    private static int binarySearch(double[] array, double key) {
        int jump = array.length >> 1;
        int insertion = jump;
        while (jump > 1) {
            if (key == array[insertion]) return insertion;
            jump >>= 1;
            if (key < array[insertion]) insertion -= jump;
            else insertion += jump;
        }
        if (key == array[insertion]) return insertion;
        return -insertion - (key > array[insertion] ? 1 : 0);
    }
}
