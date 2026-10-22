package org.firstinspires.ftc.LyonLib.trajectory;

import org.firstinspires.ftc.LyonLib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Time-Optimal Path Parameterization via Reachability Analysis (TOPP-RA),
 * ported from Team Rembrandts (FRC 4481) to pure-Java LyonLib types for FTC.
 *
 * <p>It builds a cubic Bezier path from the current pose to a target pose,
 * sampling translation and a smooth-step heading, then finds the time-optimal
 * speed profile along it subject to a single isotropic robot force limit
 * (translational and rotational lumped via the mass matrix).
 *
 * <p>Relies only on {@link Pose2d#Pose2d(double, double, double)} /
 * {@link Pose2d#toArray()} and {@link ChassisSpeeds} short fields (vx, vy).
 *
 * <p>Generate ON DEMAND (once per path request), not every loop — each call
 * allocates the path arrays.
 */
public final class ToppTrajectoryGenerator {

    private static final double EPSILON = 1e-6;
    private static final int N_POINTS = 81;
    private static final double DS = 1.0 / (N_POINTS - 1);
    private static final double[] S_VALUES = new double[N_POINTS];

    private static final double NO_ROTATION_THRESHOLD = 0.9;   // stop rotating after this s
    private static final double START_ROTATION_THRESHOLD = 0.1; // start rotating after this s

    private static final double MIN_ROBOT_SPEED = 0.4;
    private static final double END_CONTROL_POINT_LENGTH = 0.8;
    private static final double VELOCITY_TO_CONTROL_POINT_FACTOR = 0.5;
    private static final double AWAY_FROM_TARGET_ANGLE_THRESHOLD_DEG = 100.0;
    private static final double AWAY_FROM_TARGET_FACTOR = 2.0;

    private static final int SCAN_LENGTH = 14; // binary-search iterations in the forward pass

    static {
        for (int i = 0; i < N_POINTS; i++) {
            S_VALUES[i] = i * DS;
        }
    }

    private final double[] mDiag = new double[3]; // diagonal mass matrix
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;

    /**
     * @param mass      robot mass (kg)
     * @param moi       moment of inertia (kg*m^2)
     * @param wheelBase center-to-wheel half-distance along one axis (m); for a square
     *                  robot the corner distance is sqrt(2) * wheelBase
     */
    public ToppTrajectoryGenerator(
            double mass,
            double moi,
            double wheelBase,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;

        double robotRadius = Math.sqrt(2) * wheelBase;
        mDiag[0] = mass;
        mDiag[1] = mass;
        mDiag[2] = moi / robotRadius; // convert torque limit to an equivalent linear force
    }

    /**
     * @param targetPose            end pose; its heading is the FINAL facing
     * @param approachHeadingRadians direction the robot comes INTO the target from
     * @param vMax                   max linear speed (m/s)
     * @param aMax                   max linear acceleration (m/s^2)
     */
    public HolonomicTrajectory generate(
            Pose2d targetPose, double approachHeadingRadians, double vMax, double aMax) {

        double[] start = robotPoseSupplier.get().toArray(); // [x, y, heading]
        double[] target = targetPose.toArray();

        ChassisSpeeds currentSpeeds = chassisSpeedSupplier.get();
        double robotSpeed = Math.hypot(currentSpeeds.vx, currentSpeeds.vy);

        double directToTargetDir = Math.atan2(target[1] - start[1], target[0] - start[0]);
        double velocityDirection = (robotSpeed > MIN_ROBOT_SPEED)
                ? Math.atan2(currentSpeeds.vy, currentSpeeds.vx)
                : directToTargetDir;

        // Control point points backwards from the approach heading.
        double endControlPointRotation = approachHeadingRadians + Math.PI;

        double startControlPointLength =
                Math.max(END_CONTROL_POINT_LENGTH, robotSpeed * VELOCITY_TO_CONTROL_POINT_FACTOR);
        double distanceToTarget = Math.hypot(target[0] - start[0], target[1] - start[1]);
        double endControlPointLength = Math.min(END_CONTROL_POINT_LENGTH, distanceToTarget * 0.4);

        // Lengthen the start handle if we're driving away from the target (smooth U-turn).
        if (Math.abs(angleModulus(directToTargetDir - velocityDirection))
                > Math.toRadians(AWAY_FROM_TARGET_ANGLE_THRESHOLD_DEG)
                && robotSpeed > MIN_ROBOT_SPEED) {
            startControlPointLength *= AWAY_FROM_TARGET_FACTOR;
        } else {
            startControlPointLength = Math.min(startControlPointLength, distanceToTarget * 0.4);
        }

        double[][][] path = generatePathPoints(
                start, target, startControlPointLength, endControlPointLength,
                velocityDirection, endControlPointRotation);
        double[][] q = path[0];
        double[][] qPrime = path[1];
        double[][] qDoublePrime = path[2];

        double fMax = aMax * mDiag[0];

        // Initial path velocity x0 = sdot^2, from the current translational speed.
        double qPrimeStartMagSq = qPrime[0][0] * qPrime[0][0] + qPrime[0][1] * qPrime[0][1];
        double x0 = qPrimeStartMagSq > 0 ? robotSpeed * robotSpeed / qPrimeStartMagSq : 0.0;

        double[] xMax = backwardPass(qPrime, qDoublePrime, vMax, fMax, 0.0);
        double[][] result = forwardPass(qPrime, qDoublePrime, xMax, fMax, x0);
        double[] x = result[0];
        double[] u = result[1];

        return new HolonomicTrajectory(toSamples(q, qPrime, qDoublePrime, x, u));
    }

    // ---- Path geometry ----------------------------------------------------

    private static double[][][] generatePathPoints(
            double[] start,
            double[] end,
            double startControlPointLength,
            double endControlPointLength,
            double startControlPointRotation,
            double endControlPointRotation) {

        double[] p0 = {start[0], start[1]};
        double[] p3 = {end[0], end[1]};
        double[] p1 = {
                start[0] + startControlPointLength * Math.cos(startControlPointRotation),
                start[1] + startControlPointLength * Math.sin(startControlPointRotation)
        };
        double[] p2 = {
                end[0] + endControlPointLength * Math.cos(endControlPointRotation),
                end[1] + endControlPointLength * Math.sin(endControlPointRotation)
        };

        double[][] q = new double[N_POINTS][3];
        double[][] qPrime = new double[N_POINTS][3];
        double[][] qDoublePrime = new double[N_POINTS][3];

        for (int i = 0; i < N_POINTS; i++) {
            double s = S_VALUES[i];
            double[] pos = bezier(s, p0, p1, p2, p3);
            double[] vel = bezierVelocity(s, p0, p1, p2, p3);
            double[] acc = bezierAcceleration(s, p0, p1, p2, p3);

            q[i][0] = pos[0];
            q[i][1] = pos[1];
            q[i][2] = calculateRotation(s, start[2], end[2]);

            qPrime[i][0] = vel[0];
            qPrime[i][1] = vel[1];
            qPrime[i][2] = calculateRotationPrime(s, start[2], end[2]);

            qDoublePrime[i][0] = acc[0];
            qDoublePrime[i][1] = acc[1];
            qDoublePrime[i][2] = calculateRotationDoublePrime(s, start[2], end[2]);
        }

        return new double[][][] {q, qPrime, qDoublePrime};
    }

    private static double[] bezier(double s, double[] p0, double[] p1, double[] p2, double[] p3) {
        double u = 1.0 - s;
        double b0 = u * u * u;
        double b1 = 3 * u * u * s;
        double b2 = 3 * u * s * s;
        double b3 = s * s * s;
        return new double[] {
                b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
                b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]
        };
    }

    private static double[] bezierVelocity(double s, double[] p0, double[] p1, double[] p2, double[] p3) {
        double u = 1.0 - s;
        double c0 = 3 * u * u;
        double c1 = 6 * u * s;
        double c2 = 3 * s * s;
        return new double[] {
                c0 * (p1[0] - p0[0]) + c1 * (p2[0] - p1[0]) + c2 * (p3[0] - p2[0]),
                c0 * (p1[1] - p0[1]) + c1 * (p2[1] - p1[1]) + c2 * (p3[1] - p2[1])
        };
    }

    private static double[] bezierAcceleration(double s, double[] p0, double[] p1, double[] p2, double[] p3) {
        double u = 1.0 - s;
        double d0 = 6 * u;
        double d1 = 6 * s;
        return new double[] {
                d0 * (p2[0] - 2 * p1[0] + p0[0]) + d1 * (p3[0] - 2 * p2[0] + p1[0]),
                d0 * (p2[1] - 2 * p1[1] + p0[1]) + d1 * (p3[1] - 2 * p2[1] + p1[1])
        };
    }

    // ---- Heading profile (smooth-step on [START, NO_ROTATION], bugs fixed) ----

    private static double calculateRotation(double s, double startRad, double endRad) {
        double diff = angleModulus(endRad - startRad); // shortest signed angle
        if (s > NO_ROTATION_THRESHOLD) {
            return startRad + diff; // == end heading, continuous with the smooth-step
        }
        if (s < START_ROTATION_THRESHOLD) {
            return startRad;
        }
        double sp = (s - START_ROTATION_THRESHOLD) / (NO_ROTATION_THRESHOLD - START_ROTATION_THRESHOLD);
        return -2 * diff * sp * sp * sp + 3 * diff * sp * sp + startRad;
    }

    private static double calculateRotationPrime(double s, double startRad, double endRad) {
        if (s > NO_ROTATION_THRESHOLD || s < START_ROTATION_THRESHOLD) {
            return 0.0;
        }
        double diff = angleModulus(endRad - startRad);
        double span = NO_ROTATION_THRESHOLD - START_ROTATION_THRESHOLD;
        double sp = (s - START_ROTATION_THRESHOLD) / span;
        // d(theta)/ds = d(theta)/ds' * ds'/ds, with ds'/ds = 1/span (chain rule).
        return (-6 * diff * sp * sp + 6 * diff * sp) / span;
    }

    private static double calculateRotationDoublePrime(double s, double startRad, double endRad) {
        if (s > NO_ROTATION_THRESHOLD || s < START_ROTATION_THRESHOLD) {
            return 0.0;
        }
        double diff = angleModulus(endRad - startRad);
        double span = NO_ROTATION_THRESHOLD - START_ROTATION_THRESHOLD;
        double sp = (s - START_ROTATION_THRESHOLD) / span;
        return (-12 * diff * sp + 6 * diff) / (span * span);
    }

    // ---- TOPP-RA passes (unchanged from the verified original) ------------

    private double[] backwardPass(
            double[][] qPrime, double[][] qDoublePrime, double vMax, double fMax, double xEnd) {
        double[] xMax = new double[N_POINTS];
        xMax[N_POINTS - 1] = xEnd;

        final double twoDs = 2 * DS;
        final double fMaxSq = fMax * fMax;

        for (int i = N_POINTS - 2; i >= 0; i--) {
            double xNext = xMax[i + 1];

            double rNormSq = 0.0;
            double lNormSq = 0.0;
            double rDotL = 0.0;
            for (int k = 0; k < 3; k++) {
                double l = mDiag[k] * qPrime[i][k] / twoDs;
                double r = mDiag[k] * qDoublePrime[i][k] - l;
                rNormSq += r * r;
                lNormSq += l * l;
                rDotL += r * l;
            }

            double qPrimeNormSq = dot(qPrime[i], qPrime[i]);
            double xVelCap = qPrimeNormSq > EPSILON ? vMax * vMax / qPrimeNormSq : Double.POSITIVE_INFINITY;

            double xi;
            if (rNormSq < EPSILON) {
                if (lNormSq > EPSILON && lNormSq * xNext * xNext > fMaxSq) {
                    xMax[i + 1] = Math.min(xMax[i + 1], fMax / Math.sqrt(lNormSq));
                }
                xi = xVelCap;
            } else {
                double disc = 4.0 * ((rDotL * rDotL - rNormSq * lNormSq) * xNext * xNext + rNormSq * fMaxSq);
                if (disc >= 0.0) {
                    xi = (-2.0 * rDotL * xNext + Math.sqrt(disc)) / (2.0 * rNormSq);
                } else {
                    double denom = rNormSq * lNormSq - rDotL * rDotL; // >= 0 by Cauchy-Schwarz
                    double sqrtDenom = Math.sqrt(Math.max(denom, EPSILON));
                    double rNorm = Math.sqrt(rNormSq);
                    xMax[i + 1] = Math.min(xMax[i + 1], rNorm * fMax / sqrtDenom);
                    xi = -rDotL * fMax / (rNorm * sqrtDenom);
                }
            }

            xMax[i] = Math.max(0.0, Math.min(xi, xVelCap));
        }
        return xMax;
    }

    private double[][] forwardPass(
            double[][] qPrime, double[][] qDoublePrime, double[] xMax, double fMax, double x0) {
        double[] x = new double[N_POINTS];
        double[] u = new double[N_POINTS];
        x[0] = Math.min(x0, xMax[0]);

        for (int i = 1; i < N_POINTS; i++) {
            double xLow = 0;
            double xHigh = xMax[i];
            double xi = 0;
            double uRequired = 0;
            for (int j = -1; j < SCAN_LENGTH; j++) {
                xi = (xLow + xHigh) / 2;
                uRequired = (xi - x[i - 1]) / (2 * DS);

                double[][] vectors = forceVectors(qPrime[i - 1], qDoublePrime[i - 1], xi, x[i - 1]);
                double forceSq = dot(vectors[0], vectors[0]);
                double gradientDot = dot(vectors[0], vectors[1]);

                if (forceSq <= fMax * fMax) {
                    xLow = xi;
                } else if (gradientDot >= 0) {
                    xLow = xi;
                } else {
                    xHigh = xi;
                }
            }
            x[i] = xi;
            u[i] = uRequired;
        }
        return new double[][] {x, u};
    }

    private double[][] forceVectors(double[] qPrime, double[] qDoublePrime, double x, double xPrev) {
        double uRequired = (x - xPrev) / (2 * DS);
        double[][] out = new double[2][3];
        for (int i = 0; i < 3; i++) {
            double a = mDiag[i] * qPrime[i];
            double b = mDiag[i] * qDoublePrime[i];
            out[1][i] = (-a / 2 * DS);           // gradient sign-consistent with the search; only its sign is used
            out[0][i] = a * uRequired + b * xPrev; // force vector
        }
        return out;
    }

    // ---- Sample reconstruction --------------------------------------------

    private static List<HolonomicTrajectory.Sample> toSamples(
            double[][] q, double[][] qPrime, double[][] qDoublePrime, double[] x, double[] u) {
        List<HolonomicTrajectory.Sample> samples = new ArrayList<>(N_POINTS);
        double time = 0.0;
        for (int i = 0; i < N_POINTS; i++) {
            if (i > 0) {
                double v1 = Math.sqrt(x[i - 1]);
                double v2 = Math.sqrt(x[i]);
                double vAvg = (v1 + v2) / 2;
                time += (vAvg > EPSILON) ? DS / vAvg : 0.0;
            }
            double sdot = Math.sqrt(x[i]);
            // velocity = q'(s) * sdot ; acceleration = q''(s) * x + q'(s) * u
            samples.add(new HolonomicTrajectory.Sample(
                    time,
                    new Pose2d(q[i][0], q[i][1], q[i][2]),
                    qPrime[i][0] * sdot,
                    qPrime[i][1] * sdot,
                    qPrime[i][2] * sdot,
                    qDoublePrime[i][0] * x[i] + qPrime[i][0] * u[i],
                    qDoublePrime[i][1] * x[i] + qPrime[i][1] * u[i],
                    qDoublePrime[i][2] * x[i] + qPrime[i][2] * u[i]));
        }
        return samples;
    }

    // ---- helpers ----------------------------------------------------------

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double angleModulus(double radians) {
        return Math.IEEEremainder(radians, 2.0 * Math.PI);
    }
}