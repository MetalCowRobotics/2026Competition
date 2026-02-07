package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

public class shooterLookup {

    // key: distance (feet)
    // value: angle (degrees)
    private final Map<Double, Double> angleMap = new HashMap<>();

    // key: distance (feet)
    // value: velocity (RPM)
    private final Map<Double, Double> velocityMap = new HashMap<>();

    // key: distance (feet)
    // value: time of flight (seconds)
    private final Map<Double, Double> timeMap = new HashMap<>();

    public shooterLookup() {
        // Example values — replace with real tuning data
        angleMap.put(0.0,1.0);
        angleMap.put(1.0,1.0);
        angleMap.put(2.0,1.0);
        angleMap.put(3.0,1.0);
        angleMap.put(4.0,1.0);
        angleMap.put(5.0,1.0);
        angleMap.put(6.0,1.0);
        angleMap.put(7.0,1.0);
        angleMap.put(8.0,1.0);
        angleMap.put(9.0,1.0);
        angleMap.put(10.0,1.0);
        angleMap.put(11.0,1.0);
        angleMap.put(12.0,1.0);
        angleMap.put(13.0,1.0);
        angleMap.put(14.0,1.0);
        angleMap.put(15.0,1.0);
        angleMap.put(16.0,1.0);
        angleMap.put(17.0,1.0);
        angleMap.put(18.0,1.0);
        angleMap.put(19.0,1.0);
        angleMap.put(20.0,1.0);
        angleMap.put(21.0,1.0);
        angleMap.put(22.0,1.0);
        angleMap.put(23.0,1.0);
        angleMap.put(24.0,1.0);
        angleMap.put(25.0,1.0);
        angleMap.put(26.0,1.0);
        angleMap.put(27.0,1.0);
        angleMap.put(28.0,1.0);
        angleMap.put(29.0,1.0);
    
        // key: distance (feet)
        // value: velocity (rpm)

        velocityMap.put(0.0,1.0);
        velocityMap.put(1.0,1.0);
        velocityMap.put(2.0,1.0);
        velocityMap.put(3.0,1.0);
        velocityMap.put(4.0,1.0);
        velocityMap.put(5.0,1.0);
        velocityMap.put(6.0,1.0);
        velocityMap.put(7.0,1.0);
        velocityMap.put(8.0,1.0);
        velocityMap.put(9.0,1.0);
        velocityMap.put(10.0,1.0);
        velocityMap.put(11.0,1.0);
        velocityMap.put(12.0,1.0);
        velocityMap.put(13.0,1.0);
        velocityMap.put(14.0,1.0);
        velocityMap.put(15.0,1.0);
        velocityMap.put(16.0,1.0);
        velocityMap.put(17.0,1.0);
        velocityMap.put(18.0,1.0);
        velocityMap.put(19.0,1.0);
        velocityMap.put(20.0,1.0);
        velocityMap.put(21.0,1.0);
        velocityMap.put(22.0,1.0);
        velocityMap.put(23.0,1.0);
        velocityMap.put(24.0,1.0);
        velocityMap.put(25.0,1.0);
        velocityMap.put(26.0,1.0);
        velocityMap.put(27.0,1.0);
        velocityMap.put(28.0,1.0);
        velocityMap.put(29.0,1.0);

        // key: distance (feet)
        // value: time of flight (seconds)

        timeMap.put(0.0,1.0);
        timeMap.put(1.0,1.0);
        timeMap.put(2.0,1.0);
        timeMap.put(3.0,1.0);
        timeMap.put(4.0,1.0);
        timeMap.put(5.0,1.0);
        timeMap.put(6.0,1.0);
        timeMap.put(7.0,1.0);
        timeMap.put(8.0,1.0);
        timeMap.put(9.0,1.0);
        timeMap.put(10.0,1.0);
        timeMap.put(11.0,1.0);
        timeMap.put(12.0,1.0);
        timeMap.put(13.0,1.0);
        timeMap.put(14.0,1.0);
        timeMap.put(15.0,1.0);
        timeMap.put(16.0,1.0);
        timeMap.put(17.0,1.0);
        timeMap.put(18.0,1.0);
        timeMap.put(19.0,1.0);
        timeMap.put(20.0,1.0);
        timeMap.put(21.0,1.0);
        timeMap.put(22.0,1.0);
        timeMap.put(23.0,1.0);
        timeMap.put(24.0,1.0);
        timeMap.put(25.0,1.0);
        timeMap.put(26.0,1.0);
        timeMap.put(27.0,1.0);
        timeMap.put(28.0,1.0);
        timeMap.put(29.0,1.0);
    }

    /* -------------------- Basic getters -------------------- */

    private double getValue(Map<Double, Double> map, double key) {
        return map.getOrDefault(key, 0.0);
    }

    /* -------------------- Bounds helpers -------------------- */

    private double lowestKey(Map<Double, Double> map) {
        return map.keySet().stream().min(Double::compare).orElse(0.0);
    }

    private double highestKey(Map<Double, Double> map) {
        return map.keySet().stream().max(Double::compare).orElse(0.0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /* -------------------- Interpolation math -------------------- */

    public double calculateHoodAngle(double distanceFeet) {
        return interpolate(distanceFeet, angleMap);
    }

    public double calculateFlywheelVelocity(double distanceFeet) {
        return interpolate(distanceFeet, velocityMap);
    }

    public double calculateFlightTime(double distanceFeet) {
        return interpolate(distanceFeet, timeMap);
    }

    /**
     * Linear interpolation between two 1-foot spaced data points.
     */
    private double interpolate(double distanceFeet, Map<Double, Double> map) {
        double lowerBound = lowestKey(map);
        double upperBound = highestKey(map);

        double lowerDistance = clamp(Math.floor(distanceFeet), lowerBound, upperBound);
        double upperDistance = clamp(lowerDistance + 1.0, lowerBound, upperBound);

        double lowerValue = getValue(map, lowerDistance);
        double upperValue = getValue(map, upperDistance);

        // slope per inch (1 ft = 12 inches)
        double slope = (upperValue - lowerValue) / 12.0;

        // inches past the lower distance
        double inchesPastLower = (distanceFeet - lowerDistance) * 12.0;

        return slope * inchesPastLower + lowerValue;
    }
}
