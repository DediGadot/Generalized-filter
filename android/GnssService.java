// Android GNSS Service - Java Side
//
// Purpose: Register for GNSS raw measurements and forward to C++ via JNI
// Documentation: https://developer.android.com/reference/android/location/GnssMeasurementsEvent
//
// Integration:
//   1. Copy this file to your Android app: app/src/main/java/com/fusion/
//   2. Add permission to AndroidManifest.xml: ACCESS_FINE_LOCATION
//   3. Load native library in your Activity: System.loadLibrary("sensor_fusion");
//   4. Create GnssService and call start()
//
// Example Usage:
//   GnssService gnssService = new GnssService(context);
//   gnssService.start();
//   // ... GNSS data flows to C++ ...
//   gnssService.stop();
//
// Requirements:
//   - Android API 24+ (Nougat)
//   - ACCESS_FINE_LOCATION permission granted
//   - Device with GNSS raw measurement support (not all devices support this)

package com.fusion;

import android.content.Context;
import android.location.GnssMeasurement;
import android.location.GnssMeasurementsEvent;
import android.location.LocationManager;
import android.os.Build;
import android.util.Log;

import androidx.annotation.RequiresApi;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

@RequiresApi(api = Build.VERSION_CODES.N)
public class GnssService {
    private static final String TAG = "GnssService";

    private final LocationManager locationManager;
    private final GnssMeasurementsEvent.Callback gnssCallback;
    private boolean isRunning = false;

    // Native methods (implemented in jni_gnss.cpp)
    private native void nativeInit();
    private native boolean nativeStart();
    private native void nativeStop();
    private native void nativeOnMeasurement(
        long timestampNs,
        int numSats,
        int[] prns,
        double[] pseudoranges,
        float[] dopplers,
        float[] cn0s);

    static {
        // Load native library
        // Note: This assumes the library is named "libsensor_fusion.so"
        // Adjust if your library has a different name
        try {
            System.loadLibrary("sensor_fusion");
        } catch (UnsatisfiedLinkError e) {
            Log.e(TAG, "Failed to load native library", e);
        }
    }

    public GnssService(Context context) {
        locationManager = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);

        // Initialize native side
        nativeInit();

        // Create GNSS callback
        gnssCallback = new GnssMeasurementsEvent.Callback() {
            @Override
            public void onGnssMeasurementsReceived(GnssMeasurementsEvent event) {
                processGnssMeasurements(event);
            }

            @Override
            public void onStatusChanged(int status) {
                switch (status) {
                    case STATUS_READY:
                        Log.i(TAG, "GNSS measurements ready");
                        break;
                    case STATUS_NOT_SUPPORTED:
                        Log.w(TAG, "GNSS raw measurements not supported on this device");
                        break;
                    case STATUS_LOCATION_DISABLED:
                        Log.w(TAG, "Location disabled");
                        break;
                }
            }
        };
    }

    public boolean start() {
        if (isRunning) {
            Log.w(TAG, "GNSS service already running");
            return false;
        }

        if (locationManager == null) {
            Log.e(TAG, "LocationManager not available");
            return false;
        }

        // Check if device supports GNSS raw measurements
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N) {
            Log.e(TAG, "GNSS raw measurements require API 24+");
            return false;
        }

        try {
            // Register for GNSS measurements
            // Note: This requires ACCESS_FINE_LOCATION permission
            locationManager.registerGnssMeasurementsCallback(gnssCallback, null);
            isRunning = true;

            // Notify native side
            nativeStart();

            Log.i(TAG, "GNSS measurements started");
            return true;

        } catch (SecurityException e) {
            Log.e(TAG, "Missing ACCESS_FINE_LOCATION permission", e);
            return false;
        } catch (Exception e) {
            Log.e(TAG, "Failed to start GNSS measurements", e);
            return false;
        }
    }

    public void stop() {
        if (!isRunning) {
            return;
        }

        if (locationManager != null) {
            locationManager.unregisterGnssMeasurementsCallback(gnssCallback);
        }

        nativeStop();
        isRunning = false;

        Log.i(TAG, "GNSS measurements stopped");
    }

    public boolean isRunning() {
        return isRunning;
    }

    private void processGnssMeasurements(GnssMeasurementsEvent event) {
        Collection<GnssMeasurement> measurements = event.getMeasurements();
        if (measurements.isEmpty()) {
            return;
        }

        // Extract measurement data
        List<Integer> prns = new ArrayList<>();
        List<Double> pseudoranges = new ArrayList<>();
        List<Float> dopplers = new ArrayList<>();
        List<Float> cn0s = new ArrayList<>();

        long timestampNs = 0;

        for (GnssMeasurement meas : measurements) {
            // Filter for valid measurements
            if (meas.getState() == 0) {
                continue;  // Invalid state
            }

            // Get timestamp (should be same for all measurements in one epoch)
            timestampNs = meas.getReceivedSvTimeNanos();

            // Extract PRN (satellite ID)
            int prn = meas.getSvid();

            // Compute pseudorange
            // PR = (ReceivedTime - TransmitTime) * c
            // This is simplified - production code needs full clock correction
            double receivedNs = meas.getReceivedSvTimeNanos();
            double transmitNs = meas.getReceivedSvTimeNanos() -
                                meas.getPseudorangeRateMetersPerSecond() * 1e9 / 299792458.0;
            double pseudorange = (receivedNs - transmitNs) * 299792458.0 / 1e9;

            // Get Doppler shift (m/s)
            float doppler = meas.getPseudorangeRateMetersPerSecond();

            // Get carrier-to-noise ratio (dB-Hz)
            float cn0 = meas.getCn0DbHz();

            // Add to lists
            prns.add(prn);
            pseudoranges.add(pseudorange);
            dopplers.add(doppler);
            cn0s.add(cn0);
        }

        if (prns.isEmpty()) {
            return;  // No valid measurements
        }

        // Convert to arrays
        int numSats = prns.size();
        int[] prnArray = new int[numSats];
        double[] prArray = new double[numSats];
        float[] dopplerArray = new float[numSats];
        float[] cn0Array = new float[numSats];

        for (int i = 0; i < numSats; i++) {
            prnArray[i] = prns.get(i);
            prArray[i] = pseudoranges.get(i);
            dopplerArray[i] = dopplers.get(i);
            cn0Array[i] = cn0s.get(i);
        }

        // Forward to native code
        nativeOnMeasurement(timestampNs, numSats, prnArray, prArray, dopplerArray, cn0Array);

        Log.d(TAG, "Processed " + numSats + " GNSS measurements");
    }
}
