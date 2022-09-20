package org.firstinspires.ftc.teamcode.util.KalmanFilter;

/**
 * see kalmanfilter.net for thorough (and extremely easy to understand) explanation behind kalman filters... literally OP
 */

public class KalmanFilter {

    protected double Q;
    protected double R;
    protected int N;
    protected double P = 1;
    protected double K = 0;
    protected double x;
    protected SizedStack<Double> estimates;
    protected LinearRegression regression;

    // Q - Sensor Noise Covariance
    // R - Model Covariance
    // N - Number of Elements in Stack

    // Q approaches 1 as trust in sensor increase
    // R increases as trust in linear regression model increases
    // N arbitrarily used for defining previous measurements we utilize

    public KalmanFilter(double Q, double R, int N) {
        this.Q = Q;
        this.R = R;
        this.N = N;
        this.x = 0;
        this.estimates = new SizedStack<>(N);
        initializeStackWith0();
        regression = new LinearRegression(stackToDoubleArray());
        solveK();
    }

    public double update(double measurement) {
        regression.runLeastSquares();
        x += regression.predictNextValue() - estimates.peek();
        x += K * (measurement - x);
        estimates.push(x);
        regression = new LinearRegression(stackToDoubleArray());
        return x;
    }


    public void solveK() {
        for (int i = 0; i < 200; ++i) solveDARE();
    }

    public void solveDARE() {
        P = P + Q;
        K = P / (P + R);
        P = (1-K) * P;
    }

    protected void initializeStackWith0() {
        for (int i = 0; i < N; ++i) {
            estimates.push(0.0);
        }
    }

    protected double[] stackToDoubleArray() {
        double[] newValues = new double[N];
        for (int i = 0; i < estimates.size(); ++i) {
            newValues[i] = estimates.get(i);
        }
        return  newValues;
    }

}