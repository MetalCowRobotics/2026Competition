package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class AlignmentConstants {
    // PID Constants for alignment - Increased for faster response
    public static final double XY_P = 3.7;  // Increased from 1.0 to 3.0
    public static final double XY_I = 0.0;
    public static final double XY_D = 0.13;  // Added small derivative gain

    public static final double ROTATION_P = 3.0;  // Increased from 1.0 to 3.0
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.1;  // Added small derivative gain

    // Tolerance for considering alignment complete
    public static final double POSITION_TOLERANCE_METERS = 0.03; // 5 cm
    public static final double ROTATION_TOLERANCE_DEGREES = 2.0; // 2 degrees

    // Left Side Positions
    public static final Pose2d[] LEFT_POSITIONS = {
        new Pose2d(3.2, 4.18, Rotation2d.fromDegrees(0)),
        new Pose2d(3.7, 2.99, Rotation2d.fromDegrees(60)),
        new Pose2d(5.03, 2.8, Rotation2d.fromDegrees(120)),
        new Pose2d(5.83, 3.87, Rotation2d.fromDegrees(180)),
        new Pose2d(5.3, 5.07, Rotation2d.fromDegrees(240)),
        new Pose2d(3.9, 5.2, Rotation2d.fromDegrees(300)),
        new Pose2d(13.87, 5.133, Rotation2d.fromDegrees(-120)), // RC
        new Pose2d(12.504, 5.316, Rotation2d.fromDegrees(-60)),// RE
        new Pose2d(11.673, 4.196, Rotation2d.fromDegrees(0)),//RG
        new Pose2d(12.237, 2.924, Rotation2d.fromDegrees(60)),//RI
        new Pose2d(13.581, 2.753, Rotation2d.fromDegrees(120)),//RK
        new Pose2d(14.426, 3.868, Rotation2d.fromDegrees(180)),//RA
        // new Pose2d(1.166, 7.057, Rotation2d.fromDegrees(0)),
        // new Pose2d(16.44, 7.009, Rotation2d.fromDegrees(0)),
        // new Pose2d(16.44, 1.010, Rotation2d.fromDegrees(0))
    };

    // Right Side Positions
    public static final Pose2d[] RIGHT_POSITIONS = {
        new Pose2d(3.15, 3.8, Rotation2d.fromDegrees(0)),//A
        new Pose2d(4.0, 2.8, Rotation2d.fromDegrees(60)),//C
        new Pose2d(5.31, 3.02, Rotation2d.fromDegrees(120)),//E
        new Pose2d(5.8, 4.21, Rotation2d.fromDegrees(180)),//G
        new Pose2d(4.96, 5.28, Rotation2d.fromDegrees(240)),//I
        new Pose2d(3.62, 5.03, Rotation2d.fromDegrees(300)),//K
        new Pose2d(1.214, 1.006, Rotation2d.fromDegrees(225)),
        new Pose2d(13.574, 5.307, Rotation2d.fromDegrees(-120)),//RD
        new Pose2d(12.251, 5.140, Rotation2d.fromDegrees(-60)),//RF
        new Pose2d(11.682, 3.867, Rotation2d.fromDegrees(0)),//RH
        new Pose2d(12.502, 2.772, Rotation2d.fromDegrees(60)),//RJ
        new Pose2d(13.884, 2.897, Rotation2d.fromDegrees(120)),//Rl
        new Pose2d(14.431, 4.203, Rotation2d.fromDegrees(180)),//RB
        // new Pose2d(13.809, 2.985, Rotation2d.fromDegrees(0)),
        // new Pose2d(1.198, 7.013, Rotation2d.fromDegrees(135)),
        // new Pose2d(16.382, 7.009, Rotation2d.fromDegrees(0)),
        // new Pose2d(16.470, 1.096, Rotation2d.fromDegrees(0))
    };

    // Method to find closest target
    public static Pose2d findClosestLeftTarget(Pose2d currentPose) {
        Pose2d closest = LEFT_POSITIONS[0];
        double minDistance = Double.MAX_VALUE;
        
        for (Pose2d target : LEFT_POSITIONS) {
            double distance = new Translation2d(currentPose.getX(), currentPose.getY())
                .getDistance(new Translation2d(target.getX(), target.getY()));
            if (distance < minDistance) {
                minDistance = distance;
                closest = target;
            }
        }
        return closest;
    }

    public static Pose2d findClosestRightTarget(Pose2d currentPose) {
        Pose2d closest = RIGHT_POSITIONS[0];
        double minDistance = Double.MAX_VALUE;
        
        for (Pose2d target : RIGHT_POSITIONS) {
            double distance = new Translation2d(currentPose.getX(), currentPose.getY())
                .getDistance(new Translation2d(target.getX(), target.getY()));
            if (distance < minDistance) {
                minDistance = distance;
                closest = target;
            }
        }
        return closest;
    }

    public static Pose2d findClosestTarget(Pose2d currentPose) {
        // Combine your left and right target logic here
        Pose2d leftTarget = findClosestLeftTarget(currentPose);
        Pose2d rightTarget = findClosestRightTarget(currentPose);
        
        if (leftTarget == null) return rightTarget;
        if (rightTarget == null) return leftTarget;
        
        // Return the closer target
        double leftDistance = currentPose.getTranslation().getDistance(leftTarget.getTranslation());
        double rightDistance = currentPose.getTranslation().getDistance(rightTarget.getTranslation());
        
        return leftDistance < rightDistance ? leftTarget : rightTarget;
    }
} 