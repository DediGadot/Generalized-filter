package com.fusion;

/**
 * Thread Statistics and Health Metrics
 *
 * Performance and diagnostic information from fusion thread.
 */
public class ThreadStats {
    // Timing statistics
    public final long cycleCount;
    public final int avgCycleTimeUs;
    public final int maxCycleTimeUs;
    public final double currentRateHz;

    // Health indicators
    public final boolean isStationary;
    public final float cpuTemperatureC;

    // Sensor statistics
    public final long imuSamplesProcessed;
    public final long magUpdatesProcessed;

    /**
     * Constructor (called from JNI)
     */
    public ThreadStats(long cycleCount,
                       int avgCycleTimeUs,
                       int maxCycleTimeUs,
                       double currentRateHz,
                       boolean isStationary,
                       float cpuTemperatureC,
                       long imuSamplesProcessed,
                       long magUpdatesProcessed) {
        this.cycleCount = cycleCount;
        this.avgCycleTimeUs = avgCycleTimeUs;
        this.maxCycleTimeUs = maxCycleTimeUs;
        this.currentRateHz = currentRateHz;
        this.isStationary = isStationary;
        this.cpuTemperatureC = cpuTemperatureC;
        this.imuSamplesProcessed = imuSamplesProcessed;
        this.magUpdatesProcessed = magUpdatesProcessed;
    }

    /**
     * Get CPU usage estimate [%]
     *
     * Based on avg cycle time and current rate.
     */
    public double getCpuUsagePercent() {
        // CPU% = (cycle_time_us / period_us) * 100
        // period_us = 1e6 / rate_hz
        if (currentRateHz <= 0) return 0.0;
        double periodUs = 1e6 / currentRateHz;
        return (avgCycleTimeUs / periodUs) * 100.0;
    }

    @Override
    public String toString() {
        return String.format("ThreadStats{cycles=%d, avg=% dµs, max=%dµs, " +
                            "rate=%.1fHz, cpu=%.1f%%, temp=%.1f°C, " +
                            "imu=%d, mag=%d, stationary=%s}",
                cycleCount, avgCycleTimeUs, maxCycleTimeUs,
                currentRateHz, getCpuUsagePercent(), cpuTemperatureC,
                imuSamplesProcessed, magUpdatesProcessed,
                isStationary ? "yes" : "no");
    }
}
