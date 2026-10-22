package org.firstinspires.ftc.LyonLib.trajectory;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;

import java.util.List;

/**
 * A time-parameterized holonomic trajectory: an ordered list of samples, each
 * holding a timestamp, a field pose, and the field-relative velocity (SI units) and
 * acceleration (SI units) at that instant. Portable (LyonLib types only).
 *
 * <p>{@link #sample(double)} returns the (linearly interpolated) state at an
 * arbitrary time, which is what a trajectory follower reads each loop.
 */
public final class HolonomicTrajectory {

    /** One point of the trajectory. Velocities/accelerations are field-relative. */
    public static final class Sample {
        public final double timeSeconds;
        public final Pose2d pose;
        public final double vx;
        public final double vy;
        public final double omega;
        public final double ax;
        public final double ay;
        public final double alpha;
        //TODO : add jerk

        public Sample(
                double timeSeconds,
                Pose2d pose,
                double vx,
                double vy,
                double omega,
                double ax,
                double ay,
                double alpha) {
            this.timeSeconds = timeSeconds;
            this.pose = pose;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
            this.ax = ax;
            this.ay = ay;
            this.alpha = alpha;
        }
    }

    private final List<Sample> samples;

    public HolonomicTrajectory(List<Sample> samples) {
        this.samples = samples;
    }

    public List<Sample> getSamples() {
        return samples;
    }

    public double getTotalTimeSeconds() {
        return samples.isEmpty() ? 0.0 : samples.get(samples.size() - 1).timeSeconds;
    }

    public boolean isEmpty() {
        return samples.isEmpty();
    }

    /** Interpolated state at time {@code t}, clamped to the trajectory's bounds. */
    public Sample sample(double t) {
        if (samples.isEmpty()) {
            return null;
        }
        int n = samples.size();
        if (t <= samples.get(0).timeSeconds) {
            return samples.get(0);
        }
        if (t >= samples.get(n - 1).timeSeconds) {
            return samples.get(n - 1);
        }

        // Binary search for the bracketing interval [lo, hi].
        int lo = 0;
        int hi = n - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (samples.get(mid).timeSeconds <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        Sample a = samples.get(lo);
        Sample b = samples.get(hi);
        double span = b.timeSeconds - a.timeSeconds;
        double r = span > 1e-9 ? (t - a.timeSeconds) / span : 0.0;
        return interpolate(a, b, r);
    }

    private static Sample interpolate(Sample a, Sample b, double r) {
        double[] pa = a.pose.toArray();
        double[] pb = b.pose.toArray();
        double x = UtilsRBL.lerp(pa[0], pb[0], r);
        double y = UtilsRBL.lerp(pa[1], pb[1], r);
        // Interpolate heading along the shortest arc.
        double dh = Math.IEEEremainder(pb[2] - pa[2], 2.0 * Math.PI);
        double heading = pa[2] + dh * r;

        return new Sample(
                UtilsRBL.lerp(a.timeSeconds, b.timeSeconds, r),
                new Pose2d(x, y, heading),
                UtilsRBL.lerp(a.vx, b.vx, r),
                UtilsRBL.lerp(a.vy, b.vy, r),
                UtilsRBL.lerp(a.omega, b.omega, r),
                UtilsRBL.lerp(a.ax, b.ax, r),
                UtilsRBL.lerp(a.ay, b.ay, r),
                UtilsRBL.lerp(a.alpha, b.alpha, r));
    }
}