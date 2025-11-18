package com.fusion;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.LocationManager;
import android.os.Binder;
import android.os.Build;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;

import androidx.core.app.NotificationCompat;

/**
 * Fusion Service - Android Foreground Service for Real-time Sensor Fusion
 *
 * This service:
 * - Runs as foreground service (survives app backgrounding)
 * - Accesses IMU/Magnetometer via Android SensorManager
 * - Runs C++ fusion thread via JNI
 * - Provides Binder interface for apps to query pose
 * - Monitors battery/thermal health
 *
 * Usage:
 *   // Start service
 *   Intent intent = new Intent(context, FusionService.class);
 *   context.startForegroundService(intent);
 *
 *   // Bind to service
 *   ServiceConnection connection = new ServiceConnection() {
 *       public void onServiceConnected(ComponentName name, IBinder service) {
 *           FusionBinder binder = (FusionBinder) service;
 *           FusionService fusionService = binder.getService();
 *           Pose pose = fusionService.getCurrentPose();
 *       }
 *   };
 *   context.bindService(intent, connection, Context.BIND_AUTO_CREATE);
 */
public class FusionService extends Service implements SensorEventListener {
    private static final String TAG = "FusionService";
    private static final String CHANNEL_ID = "FusionServiceChannel";
    private static final int NOTIFICATION_ID = 1001;

    // Binder for IPC
    private final IBinder binder = new FusionBinder();

    // Android sensors
    private SensorManager sensorManager;
    private Sensor accelerometer;
    private Sensor gyroscope;
    private Sensor magnetometer;

    // Sensor state (for IMU fusion)
    private float[] lastGyro = new float[3];
    private float[] lastAccel = new float[3];
    private long lastGyroTime = 0;
    private long lastAccelTime = 0;
    private boolean hasGyro = false;
    private boolean hasAccel = false;

    // Wake lock (keep running when screen off)
    private PowerManager.WakeLock wakeLock;

    // Statistics
    private long startTime;
    private long imuSampleCount = 0;
    private long magSampleCount = 0;

    /**
     * Binder class for IPC
     */
    public class FusionBinder extends Binder {
        public FusionService getService() {
            return FusionService.this;
        }
    }

    // === Native Methods (JNI) ===

    static {
        System.loadLibrary("fusion");
    }

    private native boolean nativeInit();
    private native boolean nativeStart();
    private native void nativeStop();
    private native void nativeShutdown();

    private native boolean nativePushImuSample(
        long timestampNs,
        float gx, float gy, float gz,
        float ax, float ay, float az);

    private native boolean nativePushMagSample(
        long timestampNs,
        float mx, float my, float mz);

    private native Pose nativeGetPose();
    private native FusedState nativeGetState();
    private native ThreadStats nativeGetStats();
    private native void nativeSetLocation(
        double latitudeDeg,
        double longitudeDeg,
        double altitudeM);

    // === Service Lifecycle ===

    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "onCreate");

        startTime = System.currentTimeMillis();

        // Initialize C++ fusion filter
        if (!nativeInit()) {
            Log.e(TAG, "Failed to initialize native filter");
            stopSelf();
            return;
        }

        // Initialize sensors
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        if (accelerometer == null || gyroscope == null) {
            Log.e(TAG, "Required sensors not available");
            stopSelf();
            return;
        }

        // Acquire wake lock
        PowerManager powerManager = (PowerManager) getSystemService(POWER_SERVICE);
        wakeLock = powerManager.newWakeLock(
            PowerManager.PARTIAL_WAKE_LOCK,
            "FusionService::WakeLock");
        wakeLock.acquire();

        Log.i(TAG, "Service created successfully");
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.i(TAG, "onStartCommand");

        // Create notification channel (Android 8+)
        createNotificationChannel();

        // Start as foreground service
        Notification notification = createNotification();
        startForeground(NOTIFICATION_ID, notification);

        // Start fusion thread
        if (!nativeStart()) {
            Log.e(TAG, "Failed to start fusion thread");
            stopSelf();
            return START_NOT_STICKY;
        }

        // Register sensor listeners (200 Hz IMU, 50 Hz magnetometer)
        sensorManager.registerListener(this, gyroscope,
            SensorManager.SENSOR_DELAY_FASTEST);  // ~200 Hz
        sensorManager.registerListener(this, accelerometer,
            SensorManager.SENSOR_DELAY_FASTEST);  // ~200 Hz

        if (magnetometer != null) {
            sensorManager.registerListener(this, magnetometer,
                50000);  // 50 Hz (20ms period)
        }

        // TODO: Get location and set reference (from LocationManager or GPS)
        // For now, use default (San Francisco)
        nativeSetLocation(37.7749, -122.4194, 100.0);

        Log.i(TAG, "Fusion service started");

        return START_STICKY;  // Restart if killed by system
    }

    @Override
    public void onDestroy() {
        Log.i(TAG, "onDestroy");

        // Unregister sensors
        sensorManager.unregisterListener(this);

        // Stop fusion thread
        nativeStop();
        nativeShutdown();

        // Release wake lock
        if (wakeLock != null && wakeLock.isHeld()) {
            wakeLock.release();
        }

        // Log statistics
        long runTime = (System.currentTimeMillis() - startTime) / 1000;
        Log.i(TAG, String.format("Service stopped after %d seconds. " +
                                 "IMU samples: %d, Mag samples: %d",
                                 runTime, imuSampleCount, magSampleCount));

        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        return binder;
    }

    // === Sensor Callbacks ===

    @Override
    public void onSensorChanged(SensorEvent event) {
        long timestampNs = event.timestamp;

        switch (event.sensor.getType()) {
            case Sensor.TYPE_GYROSCOPE:
                lastGyro[0] = event.values[0];
                lastGyro[1] = event.values[1];
                lastGyro[2] = event.values[2];
                lastGyroTime = timestampNs;
                hasGyro = true;
                break;

            case Sensor.TYPE_ACCELEROMETER:
                lastAccel[0] = event.values[0];
                lastAccel[1] = event.values[1];
                lastAccel[2] = event.values[2];
                lastAccelTime = timestampNs;
                hasAccel = true;
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                // Push magnetometer sample directly
                nativePushMagSample(
                    timestampNs,
                    event.values[0],  // mx [µT]
                    event.values[1],  // my
                    event.values[2]   // mz
                );
                magSampleCount++;
                return;  // Early return
        }

        // Fuse IMU sample when both gyro and accel are available
        if (hasGyro && hasAccel) {
            // Use timestamp from most recent sensor
            long imuTimestamp = Math.max(lastGyroTime, lastAccelTime);

            nativePushImuSample(
                imuTimestamp,
                lastGyro[0], lastGyro[1], lastGyro[2],  // Gyro [rad/s]
                lastAccel[0], lastAccel[1], lastAccel[2]  // Accel [m/s²]
            );

            imuSampleCount++;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Handle accuracy changes if needed
        Log.d(TAG, String.format("Sensor %s accuracy changed to %d",
                                 sensor.getName(), accuracy));
    }

    // === Public API (for apps via Binder) ===

    /**
     * Get current pose (position + orientation)
     *
     * Thread-safe, low-latency (<1µs).
     *
     * @return Current pose or null if not available
     */
    public Pose getCurrentPose() {
        return nativeGetPose();
    }

    /**
     * Get full fused state (pose + velocity + covariance)
     *
     * @return Current state or null if not available
     */
    public FusedState getCurrentState() {
        return nativeGetState();
    }

    /**
     * Get thread statistics
     *
     * @return Performance and health metrics
     */
    public ThreadStats getStats() {
        return nativeGetStats();
    }

    /**
     * Check if fusion is running
     */
    public boolean isRunning() {
        ThreadStats stats = getStats();
        return stats != null && stats.cycleCount > 0;
    }

    // === Notification ===

    private void createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            NotificationChannel channel = new NotificationChannel(
                CHANNEL_ID,
                "Fusion Service",
                NotificationManager.IMPORTANCE_LOW
            );
            channel.setDescription("AR Tracking Fusion Filter");

            NotificationManager manager = getSystemService(NotificationManager.class);
            manager.createNotificationChannel(channel);
        }
    }

    private Notification createNotification() {
        // Intent to open app when notification tapped
        Intent notificationIntent = new Intent(this, getClass());
        PendingIntent pendingIntent = PendingIntent.getActivity(
            this, 0, notificationIntent,
            PendingIntent.FLAG_IMMUTABLE);

        // Get current statistics for notification
        String contentText = "Fusion active";
        ThreadStats stats = getStats();
        if (stats != null) {
            contentText = String.format("%.0f Hz, %.1f%% CPU, %.0f°C",
                stats.currentRateHz,
                stats.getCpuUsagePercent(),
                stats.cpuTemperatureC);
        }

        return new NotificationCompat.Builder(this, CHANNEL_ID)
                .setContentTitle("AR Tracking Active")
                .setContentText(contentText)
                .setSmallIcon(android.R.drawable.ic_menu_compass)  // Replace with custom icon
                .setContentIntent(pendingIntent)
                .setOngoing(true)  // Cannot be dismissed
                .build();
    }

    /**
     * Update notification with current statistics
     *
     * Called periodically to show real-time stats in notification.
     */
    public void updateNotification() {
        Notification notification = createNotification();
        NotificationManager manager = getSystemService(NotificationManager.class);
        manager.notify(NOTIFICATION_ID, notification);
    }
}
