package frc.robot.constants;

public class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 17; // TODO: Adjust this ID
    public static final int LEAD_MOTOR_ID = 0; // TODO: Adjust this ID
    public static final int FOLLOW_MOTOR_ID = 0; // TODO: Adjust this ID
    
    public static final double INTAKE_SPEED = 0.9; // TODO: Adjust this speed
    public static final double REVERSE_SPEED = -0.9; // TODO: Adjust this speed
    public static final double INTAKE_OUT = 0.10; // TODO: Adjust this position
    public static final double INTAKE_IN = 0; // TODO: Adjust this position
    public static final double TIME_BETWEEN_AGITATION = 0.4;

    public static final double METERS_PER_ROTATION = 0.1595; // π * 0.0508m (circumference of 2-inch sprocket)


    public static final double KS = 0.25;
    public static final double KV = 0.02;
    public static final double KA = 0.01;
    public static final double KP = 10;
    public static final double KI = 0;
    public static final double KD = 1.0;

    
    public static final double MOTION_MAGIC_ACCELERATION = 59.54;
    public static final double CRUISE_VELOCITY = 39.27;
    public static final double MOTION_MAGIC_JERK = 100.08;
    
}
