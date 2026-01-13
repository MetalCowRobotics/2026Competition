package frc.robot.constants;

public final class WristConstants {
    public static final double L4_Angle = 8.4;
    public static final double L3_Angle = 7.2;
    public static final double Source_Angle = 4.2;
    public static final double Rest_Angle = 0;

    // Motor configuration
    public static final int WRIST_MOTOR_ID = 19; // Updated to correct CAN ID
    public static final double WRIST_GEAR_RATIO = 100.0; // Adjust based on your gear reduction
    
    // PID Constants
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    
    // Motion constraints
    public static final double MAX_VELOCITY = 100; // deg/s
    public static final double MAX_ACCELERATION = 200; // deg/s^2
    
    // Soft limits
    public static final double FORWARD_SOFT_LIMIT = 30; // degrees
    public static final double REVERSE_SOFT_LIMIT = -5; // degrees
} 