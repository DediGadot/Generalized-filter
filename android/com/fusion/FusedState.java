package com.fusion;

/**
 * Complete Fused State (Pose + Velocity + Covariance)
 *
 * Full filter output including velocity and uncertainty.
 * Used for apps that need velocity or covariance information.
 */
public class FusedState {
    // Position in navigation frame [m]
    public final double[] position;

    // Velocity in navigation frame [m/s]
    public final double[] velocity;

    // Orientation quaternion (body → navigation) [x, y, z, w]
    public final double[] orientation;

    // Uncertainty (1-sigma std dev)
    public final double[] positionStd;
    public final double[] velocityStd;
    public final double[] attitudeStd;

    // Metadata
    public final long timestampNs;
    public final long sequence;

    /**
     * Constructor (called from JNI)
     */
    public FusedState(double[] position,
                      double[] velocity,
                      double[] orientation,
                      double[] positionStd,
                      double[] velocityStd,
                      double[] attitudeStd,
                      long timestampNs,
                      long sequence) {
        this.position = position;
        this.velocity = velocity;
        this.orientation = orientation;
        this.positionStd = positionStd;
        this.velocityStd = velocityStd;
        this.attitudeStd = attitudeStd;
        this.timestampNs = timestampNs;
        this.sequence = sequence;
    }

    /**
     * Convert to lightweight Pose
     */
    public Pose toPose() {
        return new Pose(
            position[0], position[1], position[2],
            orientation[0], orientation[1], orientation[2], orientation[3],
            timestampNs
        );
    }

    /**
     * Get position uncertainty magnitude [m]
     */
    public double getPositionUncertainty() {
        return Math.sqrt(positionStd[0] * positionStd[0] +
                         positionStd[1] * positionStd[1] +
                         positionStd[2] * positionStd[2]);
    }

    /**
     * Get velocity uncertainty magnitude [m/s]
     */
    public double getVelocityUncertainty() {
        return Math.sqrt(velocityStd[0] * velocityStd[0] +
                         velocityStd[1] * velocityStd[1] +
                         velocityStd[2] * velocityStd[2]);
    }

    @Override
    public String toString() {
        return String.format("FusedState{pos=[%.3f, %.3f, %.3f], " +
                            "vel=[%.3f, %.3f, %.3f], " +
                            "posStd=%.3f, velStd=%.3f, seq=%d}",
                position[0], position[1], position[2],
                velocity[0], velocity[1], velocity[2],
                getPositionUncertainty(), getVelocityUncertainty(),
                sequence);
    }
}
