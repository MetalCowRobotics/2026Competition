package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class PivotLookup {

    // key: distance (meters)
    // value: angle (degrees)
    private final TreeMap<Double, Double> angleMap = new TreeMap<>();

    public PivotLookup() {
        // Real tuning data (distance in meters → angle in degrees)
        angleMap.put(1.252, 15.0);
        angleMap.put(1.65, 20.0);
        angleMap.put(1.93, 25.0);
        angleMap.put(2.21, 30.0);
    }

    /* -------------------- Main API -------------------- */

    public double calculateHoodAngle(double distanceMeters) {
        return interpolate(distanceMeters);
    }

    /* -------------------- Interpolation -------------------- */

    private double interpolate(double distanceMeters) {
        // If exact match exists, return it
        if (angleMap.containsKey(distanceMeters)) {
            return angleMap.get(distanceMeters);
        }

        // Get nearest lower and upper keys
        Double lowerKey = angleMap.floorKey(distanceMeters);
        Double upperKey = angleMap.ceilingKey(distanceMeters);

        // Handle out-of-bounds
        if (lowerKey == null) return angleMap.get(upperKey);
        if (upperKey == null) return angleMap.get(lowerKey);

        double lowerValue = angleMap.get(lowerKey);
        double upperValue = angleMap.get(upperKey);

        // Standard linear interpolation
        double t = (distanceMeters - lowerKey) / (upperKey - lowerKey);
        return lowerValue + t * (upperValue - lowerValue);
    }
}