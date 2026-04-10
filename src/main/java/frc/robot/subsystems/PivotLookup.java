package frc.robot.subsystems;

import java.util.TreeMap;

public class PivotLookup {

    // Both keyed on distance (meters), interpolated independently
    private final TreeMap<Double, Double> angleMap    = new TreeMap<>();
    private final TreeMap<Double, Double> velocityMap = new TreeMap<>();
    

    public PivotLookup() {
        // Distance → Angle (degrees) angleMap.put(1.72, 15.0);
        // angleMap.put(2.26, 20.0);
        // angleMap.put(3.05, 25.0);
        // angleMap.put(3.40, 30.0);
        // angleMap.put(4.25, 35.0);  // Plateau — angle holds, power still climbs
        // angleMap.put(4.75, 30.0);
        // angleMap.put(5.25, 35.0);
        angleMap.put(1.45, 19.0);
        angleMap.put(1.72, 19.0);
        angleMap.put(2.26, 23.0);
        angleMap.put(3.05, 29.0);
        angleMap.put(3.40, 33.0);
        angleMap.put(4.25, 38.0);  // Plateau — angle holds, power still climbs
        angleMap.put(4.75, 33.0);
        angleMap.put(5.25, 38.0);

        // Distance → Shooter power (percent, 0.0–1.0)
        velocityMap.put(1.45, 0.7);
        velocityMap.put(1.72, 0.7);
        velocityMap.put(2.46, 0.75);
        velocityMap.put(3.05, 0.75);
        velocityMap.put(3.45, 0.75);
        velocityMap.put(3.75, 0.78);
        
        velocityMap.put(4.25, 0.83);
        velocityMap.put(4.75, 1.0);
        velocityMap.put(5.25, 1.0);
    }

    /* -------------------- Main API -------------------- */

    public double getAngle(double distanceMeters) {
        return interpolate(angleMap, distanceMeters);
    }

    public double getPower(double distanceMeters) {
        return interpolate(velocityMap, distanceMeters);
    }

    /* -------------------- Interpolation -------------------- */

    private double interpolate(TreeMap<Double, Double> map, double distanceMeters) {
        if (map.containsKey(distanceMeters)) return map.get(distanceMeters);

        Double lowerKey = map.floorKey(distanceMeters);
        Double upperKey = map.ceilingKey(distanceMeters);

        // Clamp to table bounds instead of returning null
        if (lowerKey == null) return map.get(upperKey);
        if (upperKey == null) return map.get(lowerKey);

        double lowerValue = map.get(lowerKey);
        double upperValue = map.get(upperKey);

        double t = (distanceMeters - lowerKey) / (upperKey - lowerKey);
        return lowerValue + t * (upperValue - lowerValue);
    }
}