package com.fusion;

/**
 * Lightweight 6DOF Pose (Position + Orientation)
 *
 * Represents the current position and orientation of the device.
 * Used for apps that only need pose (no velocity or covariance).
 */
public class Pose {
    // Position in navigation frame [m]
    public final double px, py, pz;

    // Orientation quaternion (body → navigation)
    public final double qx, qy, qz, qw;

    // Timestamp [nanoseconds]
    public final long timestampNs;

    /**
     * Constructor (called from JNI)
     */
    public Pose(double px, double py, double pz,
                double qx, double qy, double qz, double qw,
                long timestampNs) {
        this.px = px;
        this.py = py;
        this.pz = pz;
        this.qx = qx;
        this.qy = qy;
        this.qz = qz;
        this.qw = qw;
        this.timestampNs = timestampNs;
    }

    /**
     * Convert quaternion to Euler angles (roll, pitch, yaw) [radians]
     */
    public double[] toEuler() {
        double[] euler = new double[3];

        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        euler[0] = Math.atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2 * (qw * qy - qz * qx);
        if (Math.abs(sinp) >= 1) {
            euler[1] = Math.copySign(Math.PI / 2, sinp); // Use 90 degrees if out of range
        } else {
            euler[1] = Math.asin(sinp);
        }

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        euler[2] = Math.atan2(siny_cosp, cosy_cosp);

        return euler;
    }

    @Override
    public String toString() {
        return String.format("Pose{pos=[%.3f, %.3f, %.3f], quat=[%.3f, %.3f, %.3f, %.3f], ts=%d}",
                px, py, pz, qx, qy, qz, qw, timestampNs);
    }
}
