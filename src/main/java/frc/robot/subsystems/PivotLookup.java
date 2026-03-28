package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class PivotLookup {

    // key: distance (meters)
    // value: angle (degrees)
    private final TreeMap<Double, Double> angleMap = new TreeMap<>();
    private final TreeMap<Double, Double> velocityMap = new TreeMap<>();

    public PivotLookup() {
        // Real tuning data (distance in meters → angle in degrees)
        angleMap.put(1.74, 15.0);
        angleMap.put(2.46, 20.0);
        angleMap.put(3.35, 25.0);
        angleMap.put(3.75, 30.0);
        

        velocityMap.put(.6, 15.0);
        velocityMap.put(.6, 20.0);

        /*15 1.74 .55 
20 2.46 .55 
25 3.35 .60
30 3.75 .65

30 4.25 .70  
30 4.75 .75

35 5.25 .85

 */
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