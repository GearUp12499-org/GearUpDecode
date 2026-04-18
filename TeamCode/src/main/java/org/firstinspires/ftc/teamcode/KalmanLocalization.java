package org.firstinspires.ftc.teamcode;

public class KalmanLocalization {

    //  State: [x, y, theta]  (inches and radians)

    public static double[][] currentState = {
            {0},  // x (inches)
            {0},  // y (inches)
            {0},  // theta (radians)
    };

    // 3x3 state covariance
    public static double[][] currentP = {
            {1, 0, 0},
            {0, 2, 0},
            {0, 0, 3},
    };

    // Process noise Q — increase to trust the motion model less
    static double[][] Q = {
            {0.1, 0,   0  },
            {0,   0.1, 0  },
            {0,   0,   0.1},
    };

    // Limelight measurement noise R — increase to trust Limelight less
    static double xNoiseLL = 0.1;
    static double yNoiseLL = 0.5;
    static double[][] R_LL = {
            {xNoiseLL, 0       },
            {0,        yNoiseLL},
    };

    // Limelight observation Jacobian C 2x3
    static double[][] C = {
            {0.94487, 0,       0},
            {0,       0.99711, 0},
    };


    public static void extendedKalman(
            double newXPinpoint, double newYPinpoint, double newAPinpoint,
            double newXVel,      double newYVel,      double newAngularVel,
            double newXLL,       double newYLL,
            double loopTime) {

        double dt   = loopTime / 1000.0;
        double cosA = Math.cos(newAPinpoint);
        double sinA = Math.sin(newAPinpoint);

        // State derivative: body velocities rotated to world frame
        double[][] f = {
                {newXVel * cosA - newYVel * sinA},
                {newXVel * sinA + newYVel * cosA},
                {newAngularVel},
        };
        double[][] predictedState = matAdd(currentState, matScale(f, dt));

        // Jacobian A [x, y, theta]
        double[][] A = {
                {0, 0, -newXVel * sinA - newYVel * cosA},
                {0, 0,  newXVel * cosA - newYVel * sinA},
                {0, 0,  0},
        };
        double[][] AT   = transpose(A);
        double[][] Pdot = matAdd(matAdd(matMul(A, currentP), matMul(currentP, AT)), Q);
        double[][] predictedP = matAdd(currentP, matScale(Pdot, dt));

        //Update Limelight

        double px = predictedState[0][0];
        double py = predictedState[1][0];
        double[][] h = {
                {px + (-0.0551323  * px + 2.19256 )},
                {py + (-0.00289434 * py + 0.576114)},
        };

        // Kalman gain is K = P * Cᵀ * (C * P * Cᵀ + R)^-1
        double[][] CT = transpose(C);
        double[][] S  = matAdd(matMul(matMul(C, predictedP), CT), R_LL);
        double[][] K  = matMul(matMul(predictedP, CT), invert2x2(S));
        //state and covar update
        double[][] z          = {{newXLL}, {newYLL}};
        double[][] innovation = matSub(z, h);
        currentState = matAdd(predictedState, matMul(K, innovation));
        currentP     = matMul(matSub(identity(3), matMul(K, C)), predictedP);
    }

    //  predictOnly(when Limelight has no valid reads) runs the predict step only, skipping the measurement update
    public static void predictOnly(
            double newAPinpoint,
            double newXVel, double newYVel, double newAngularVel,
            double loopTime) {

        double dt   = loopTime / 1000.0;
        double cosA = Math.cos(newAPinpoint);
        double sinA = Math.sin(newAPinpoint);

        double[][] f = {
                {newXVel * cosA - newYVel * sinA},
                {newXVel * sinA + newYVel * cosA},
                {newAngularVel},
        };
        currentState = matAdd(currentState, matScale(f, dt));

        double[][] A = {
                {0, 0, -newXVel * sinA - newYVel * cosA},
                {0, 0,  newXVel * cosA - newYVel * sinA},
                {0, 0,  0},
        };
        double[][] AT   = transpose(A);
        double[][] Pdot = matAdd(matAdd(matMul(A, currentP), matMul(currentP, AT)), Q);
        currentP = matAdd(currentP, matScale(Pdot, dt));
    }

    //  firstFilter is exponential low-pass smoother for Pinpoint readings
    //  Call before passing Pinpoint values into extendedKalman()
    public static double firstFilter(double value, double prevValue, double loopTime) {
        double dt          = loopTime / 1000.0;
        double timeConstant = 0.2;  // seconds — tune as needed
        double alpha       = dt / (timeConstant + dt);
        return alpha * value + (1.0 - alpha) * prevValue;
    }

    //  resetState
    public static void resetState(double startX, double startY, double startTheta) {
        currentState = new double[][]{{startX}, {startY}, {startTheta}};
        currentP     = new double[][]{{1,0,0},{0,2,0},{0,0,3}};
    }

    public static double[] getEstimate() {
        return new double[]{ currentState[0][0], currentState[1][0], currentState[2][0] };
    }


    //  Matrix helpers

    static double[][] matMul(double[][] A, double[][] B) {
        int m = A.length, k = A[0].length, n = B[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                for (int p = 0; p < k; p++)
                    C[i][j] += A[i][p] * B[p][j];
        return C;
    }

    static double[][] matAdd(double[][] A, double[][] B) {
        int m = A.length, n = A[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    static double[][] matSub(double[][] A, double[][] B) {
        int m = A.length, n = A[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[i][j] = A[i][j] - B[i][j];
        return C;
    }

    static double[][] matScale(double[][] A, double s) {
        int m = A.length, n = A[0].length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[i][j] = A[i][j] * s;
        return C;
    }

    static double[][] transpose(double[][] A) {
        int m = A.length, n = A[0].length;
        double[][] C = new double[n][m];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[j][i] = A[i][j];
        return C;
    }

    static double[][] identity(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) I[i][i] = 1.0;
        return I;
    }

    static double[][] invert2x2(double[][] M) {
        double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        if (Math.abs(det) < 1e-12)
            throw new ArithmeticException("Singular matrix — check R_LL noise values");
        double inv = 1.0 / det;
        return new double[][]{
                { M[1][1] * inv, -M[0][1] * inv},
                {-M[1][0] * inv,  M[0][0] * inv},
        };
    }
}