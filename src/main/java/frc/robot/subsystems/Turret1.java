package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.*;


public class Turret1 extends SubsystemBase {

    private final TalonFX turretMotor;
    private final TalonFX pivotMotor;
    private final CommandSwerveDrivetrain drivetrain;

    // Control Constants
    private final double KP1 = 0.09; 
     private final double KP2 = 0.006; 
    private final double TURRET_GEAR_RATIO = 11.0;
    private final double PIVOT_GEAR_RATIO = -50.28571428571429; // Your custom multiplier
    
    private final PIDController turretPID = new PIDController(KP2, 0.0, 0.0);
    private final PIDController pivotPID = new PIDController(KP1, 0.0, 0.0);

    // Physics Constants (Standardized to Feet for the formula)
    private final double TARGET_HEIGHT_DIFF = 1.27;
    private final double SHOOTER_SPEED = 10.77;
    private final double GRAVITY = 9.81;

    private final SlewRateLimiter pivotRamp = new SlewRateLimiter(0.5);

    // Field Constants (Converted to Meters for WPILib compatibility)
    private final Translation2d redHubMeters = new Translation2d(
        Units.inchesToMeters(469.11), 
        Units.inchesToMeters(158.845)
    );

    public Turret1(CommandSwerveDrivetrain drivetrain) {
        this.turretMotor = new TalonFX(30);
        this.pivotMotor = new TalonFX(14);
        this.drivetrain = drivetrain;

        turretPID.enableContinuousInput(-180.0, 180.0);
        turretPID.setTolerance(3.0); // Stop moving if within 3 degrees

        // Pivot does not wrap (it has physical limits), but we can set a tolerance
        // Tolerance is in motor rotations (approx 0.5 degrees at the mechanism)
        double pivotToleranceRotations = (0.5 / 360.0) * PIVOT_GEAR_RATIO;
        pivotPID.setTolerance(Math.abs(pivotToleranceRotations));

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO; 
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretConfig.CurrentLimits.StatorCurrentLimit = 25;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 20;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;

        // Software Limit Switches for Pivot (Safety logic)
        // Converts 0-45 degrees (scaled) into motor rotations

        //         // Resets the internal encoder positions to 0.0 rotations
        //turretMotor.setPosition(0.0);
       zeroSensors();
      
      
        turretMotor.getConfigurator().apply(turretConfig);
        pivotMotor.getConfigurator().apply(pivotConfig);

    }
    
/**
 * Manually resets encoder positions to zero. 
 * Use this when the turret and pivot are physically at their "home" positions.
 */


public void zeroSensors() {
    turretMotor.setPosition(0);
    pivotMotor.setPosition(turretMotor.getPosition().getValueAsDouble());
}

// public void autoGoal()
// {
//     Pose2d robotPose = drivetrain.getState().Pose;
//     // Apply turret offset to robot pose
//     Transform2d turretOffset = 
//       new Transform2d(
//         Units.inchesToMeters(-6),
//         Units.inchesToMeters(0),
//         new Rotation2d());
//     Pose2d turretPose = robotPose.plus(turretOffset);
    
//     // Calculate vector to target
//     double dY = redHubMeters.getY() - turretPose.getY();
//     double dX = redHubMeters.getX() - turretPose.getX();
    
//     // Calculate field-relative angle to target
//     Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(dY, dX));
    
//     // Convert to robot-relative angle
//     Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

//     double beginTurretPose = turretMotor.getPosition().getValueAsDouble();

//     // Update angle variables
//     // m_robotRelativeAngle = robotRelativeAngle.getDegrees();
//     // m_fieldRelativeAngle = fieldRelativeAngle.getDegrees();
    
//     // Command turret
//     double moddedPosition = MathUtil.inputModulus(robotRelativeAngle.getDegrees(), 0, 360);

//     double turretError = MathUtil.inputModulus(robotRelativeAngle.getDegrees() - beginTurretPose, -180, 180);
    
//     turretMotor.set(Math.abs(turretError) < 3 ? 0 : turretError * KP2);

//     //turretMotor.setPosition(moddedPosition/360 * k);
// }

public void autoGoal()
{
    Pose2d robotPos = drivetrain.getState().Pose;

    double beginTurretPose = turretMotor.getPosition().getValueAsDouble();
    
    // --- 1. TURRET OFFSET MATH ---
    // Define the turret's physical offset from the center of the robot.
    // TODO: Replace 10.0 and 0.0 with your actual measurements in inches!
    Translation2d robotToTurretOffset = new Translation2d(
        Units.inchesToMeters(-6.0), // X offset (Positive is forward of center)
        Units.inchesToMeters(0.0)   // Y offset (Positive is left of center)
    );
    
    // Rotate the offset by the robot's current heading to make it field-relative
    Translation2d fieldRelativeOffset = robotToTurretOffset.rotateBy(robotPos.getRotation());
    
    // Add the field-relative offset to the robot's center position to get the turret's true field position
    Translation2d turretFieldPos = robotPos.getTranslation().plus(fieldRelativeOffset);

    // Get robot-relative speeds and transform them to field-relative
    var robotRelativeSpeeds = drivetrain.getState().Speeds;
    var fieldRelativeSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, 
        robotPos.getRotation()
    );

    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

    // --- 2. PREDICTIVE TARGETING (Motion Compensation) ---
    // Note: Now using turretFieldPos instead of robotPos.getTranslation()
    double distToHub = turretFieldPos.getDistance(redHubMeters);
    double v = SHOOTER_SPEED;
    double flightTime = distToHub / v; // Rough estimate of time-to-target

    // Calculate lead position: Target = Current - (Velocity * Time)
    Translation2d leadingTarget = new Translation2d(
        redHubMeters.getX() - (robotVx * flightTime),
        redHubMeters.getY() - (robotVy * flightTime)
    );

    // --- 3. PROJECTILE MATH ---
    // Note: Again, using turretFieldPos for distance to the lead target
    double distanceToLead = turretFieldPos.getDistance(leadingTarget);
    double g = GRAVITY;
    double h = TARGET_HEIGHT_DIFF;

    double v2 = v * v;
    double v4 = v2 * v2;
    double discriminant = v4 - g * (g * (distanceToLead * distanceToLead) + 2 * h * v2);

    // SAFETY CHECK: Prevent Math.sqrt(negative_number) returning NaN
    if (discriminant < 0) {
        discriminant = 0; 
        // Note: You might want to add a boolean flag here to tell the shooter 
        // not to fire, because discriminant < 0 means the target is out of range!
    }

    // --- 4. TURRET CONTROL (Azimuth) ---
    // double turretRotations = turretMotor.getPosition().getValueAsDouble();
    // double currentTurretAngle = (turretRotations * 360.0) % 360.0;
    // if (currentTurretAngle < 0) currentTurretAngle += 360.0;

    // // Calculate angle to the LEADING target using the TURRET'S field position
    // double angleToLeadRad = Math.atan2(
    //     leadingTarget.getY() - turretFieldPos.getY(), 
    //     leadingTarget.getX() - turretFieldPos.getX()
    // );
    
    // double turretTargetDeg = Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees();
    // double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
    
    // turretMotor.set(Math.abs(turretError) < 3 ? 0 : turretError * KP2);

    // --- 4. TURRET CONTROL (Azimuth) ---
double turretRotations = turretMotor.getPosition().getValueAsDouble();
    double currentTurretAngle = MathUtil.inputModulus(turretRotations * 360.0, -180, 180);

    double angleToLeadDeg = Units.radiansToDegrees(
        Math.atan2(leadingTarget.getY() - turretFieldPos.getY(), 
                   leadingTarget.getX() - turretFieldPos.getX())
    );

    double turretTargetDeg = MathUtil.inputModulus(
        angleToLeadDeg - robotPos.getRotation().getDegrees(), 
        -180, 180
    );

    // Let the WPILib PIDController do the heavy lifting!
    // Because we enabled continuous input, it automatically knows the shortest path.
    double turretOutput = turretPID.calculate(currentTurretAngle, turretTargetDeg);

    // Only apply power if we aren't already at the target
    turretMotor.set(turretPID.atSetpoint() ? 0.0 : turretOutput);

    // --- 5. PIVOT CONTROL (Coaxial Compensation) ---

double targetPitchDegrees;
double angleRad = Math.atan((v2 - Math.sqrt(discriminant)) / (g * distanceToLead));
targetPitchDegrees = MathUtil.clamp(Math.toDegrees(angleRad), 2.0, 40.0);

/**
 * 1. PITCH COMPONENT
 * Calculate motor rotations needed to tilt the shooter up/down.
 */

    // 1. Pitch Component (in Motor Rotations)
    double pivotRotationsForPitch = (targetPitchDegrees / 360.0) * PIVOT_GEAR_RATIO;

    // 2. Coaxial Component (in Motor Rotations)
    double turretMechanismRotations = turretRotations / TURRET_GEAR_RATIO;
    double coaxialCompensationRotations = turretMechanismRotations * PIVOT_GEAR_RATIO;

    // 3. Final Setpoint
    double finalPivotTargetRotations = pivotRotationsForPitch + coaxialCompensationRotations;

    // 4. Current Position
    double currentPivotRotations = pivotMotor.getPosition().getValueAsDouble();

    // 5. Calculate Output using PID
    double pivotOutput = pivotPID.calculate(currentPivotRotations, finalPivotTargetRotations);

    // Apply power
    pivotMotor.set(pivotPID.atSetpoint() ? 0.0 : pivotOutput);

    // --- 6. TELEMETRY ---
    SmartDashboard.putNumber("Turret/Dist_To_Lead", distanceToLead);
    // SmartDashboard.putNumber("Turret/Pivot_Error", pivotError);
    SmartDashboard.putNumber("Turret/Current_Pitch_Deg", currentPivotRotations*360);
    SmartDashboard.putNumber("Turret/Target_Pitch_Deg", targetPitchDegrees);
    // SmartDashboard.putNumber("Turret/Combined_Target_Pitch_Deg", -combinedPivotTargetDeg);
    SmartDashboard.putNumber("Turret/Current_Angle_Deg", currentTurretAngle);
    SmartDashboard.putNumber("Turret/Target_Angle_Deg", turretTargetDeg);
}
   


@Override
public void periodic() {   
    autoGoal();
}

}




// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.*;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.*;


// public class Turret1 extends SubsystemBase {

//     private final TalonFX turretMotor;
//     private final TalonFX pivotMotor;
//     private final CommandSwerveDrivetrain drivetrain;

//     // Control Constants
//     private final double KP1 = 0.09; 
//      private final double KP2 = 0.006; 
//     private final double TURRET_GEAR_RATIO = 11.0;
//     private final double PIVOT_GEAR_RATIO = -50.28571428571429; // Your custom multiplier
    
//     private final PIDController turretPID = new PIDController(KP2, 0.0, 0.0);
//     private final PIDController pivotPID = new PIDController(KP1, 0.0, 0.0);

//     // Physics Constants (Standardized to Feet for the formula)
//     private final double TARGET_HEIGHT_DIFF = 1.27;
//     private final double SHOOTER_SPEED = 10.77;
//     private final double GRAVITY = 9.81;

//     private final SlewRateLimiter pivotRamp = new SlewRateLimiter(0.5);

//     // Field Constants (Converted to Meters for WPILib compatibility)
//     private final Translation2d redHubMeters = new Translation2d(
//         Units.inchesToMeters(469.11), 
//         Units.inchesToMeters(158.845)
//     );

//     public Turret1(CommandSwerveDrivetrain drivetrain) {
//         this.turretMotor = new TalonFX(30);
//         this.pivotMotor = new TalonFX(14);
//         this.drivetrain = drivetrain;

//         turretPID.enableContinuousInput(-180.0, 180.0);
//         turretPID.setTolerance(3.0); // Stop moving if within 3 degrees

//         // Pivot does not wrap (it has physical limits), but we can set a tolerance
//         // Tolerance is in motor rotations (approx 0.5 degrees at the mechanism)
//         double pivotToleranceRotations = (0.5 / 360.0) * PIVOT_GEAR_RATIO;
//         pivotPID.setTolerance(Math.abs(pivotToleranceRotations));

//         configureMotors();
//     }

//     private void configureMotors() {
//         TalonFXConfiguration turretConfig = new TalonFXConfiguration();
//         turretConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO; 
//         turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
//         turretConfig.CurrentLimits.StatorCurrentLimit = 25;
//         turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

//         TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
//         pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
//         pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//         pivotConfig.CurrentLimits.StatorCurrentLimit = 20;
//         pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
//         pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;

//         // Software Limit Switches for Pivot (Safety logic)
//         // Converts 0-45 degrees (scaled) into motor rotations

//         //         // Resets the internal encoder positions to 0.0 rotations
//         //turretMotor.setPosition(0.0);
//        zeroSensors();
      
      
//         turretMotor.getConfigurator().apply(turretConfig);
//         pivotMotor.getConfigurator().apply(pivotConfig);

//     }
    
// /**
//  * Manually resets encoder positions to zero. 
//  * Use this when the turret and pivot are physically at their "home" positions.
//  */


// public void zeroSensors() {
//     turretMotor.setPosition(180);
//     pivotMotor.setPosition(turretMotor.getPosition().getValueAsDouble());
// }

// // public void autoGoal()
// // {
// //     Pose2d robotPose = drivetrain.getState().Pose;
// //     // Apply turret offset to robot pose
// //     Transform2d turretOffset = 
// //       new Transform2d(
// //         Units.inchesToMeters(-6),
// //         Units.inchesToMeters(0),
// //         new Rotation2d());
// //     Pose2d turretPose = robotPose.plus(turretOffset);
    
// //     // Calculate vector to target
// //     double dY = redHubMeters.getY() - turretPose.getY();
// //     double dX = redHubMeters.getX() - turretPose.getX();
    
// //     // Calculate field-relative angle to target
// //     Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(dY, dX));
    
// //     // Convert to robot-relative angle
// //     Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

// //     double beginTurretPose = turretMotor.getPosition().getValueAsDouble();

// //     // Update angle variables
// //     // m_robotRelativeAngle = robotRelativeAngle.getDegrees();
// //     // m_fieldRelativeAngle = fieldRelativeAngle.getDegrees();
    
// //     // Command turret
// //     double moddedPosition = MathUtil.inputModulus(robotRelativeAngle.getDegrees(), 0, 360);

// //     double turretError = MathUtil.inputModulus(robotRelativeAngle.getDegrees() - beginTurretPose, -180, 180);
    
// //     turretMotor.set(Math.abs(turretError) < 3 ? 0 : turretError * KP2);

// //     //turretMotor.setPosition(moddedPosition/360 * k);
// // }

// public void autoGoal()
// {
//     Pose2d robotPos = drivetrain.getState().Pose;

//     double beginTurretPose = turretMotor.getPosition().getValueAsDouble();
    
//     // --- 1. TURRET OFFSET MATH ---
//     // Define the turret's physical offset from the center of the robot.
//     // TODO: Replace 10.0 and 0.0 with your actual measurements in inches!
//     Translation2d robotToTurretOffset = new Translation2d(
//         Units.inchesToMeters(-6.0), // X offset (Positive is forward of center)
//         Units.inchesToMeters(0.0)   // Y offset (Positive is left of center)
//     );
    
//     // Rotate the offset by the robot's current heading to make it field-relative
//     Translation2d fieldRelativeOffset = robotToTurretOffset.rotateBy(robotPos.getRotation());
    
//     // Add the field-relative offset to the robot's center position to get the turret's true field position
//     Translation2d turretFieldPos = robotPos.getTranslation().plus(fieldRelativeOffset);

//     // Get robot-relative speeds and transform them to field-relative
//     var robotRelativeSpeeds = drivetrain.getState().Speeds;
//     var fieldRelativeSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
//         robotRelativeSpeeds, 
//         robotPos.getRotation()
//     );

//     double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
//     double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

//     // --- 2. PREDICTIVE TARGETING (Motion Compensation) ---
//     // Note: Now using turretFieldPos instead of robotPos.getTranslation()
//     double distToHub = turretFieldPos.getDistance(redHubMeters);
//     double v = SHOOTER_SPEED;
//     double flightTime = distToHub / v; // Rough estimate of time-to-target

//     // Calculate lead position: Target = Current - (Velocity * Time)
//     Translation2d leadingTarget = new Translation2d(
//         redHubMeters.getX() - (robotVx * flightTime),
//         redHubMeters.getY() - (robotVy * flightTime)
//     );

//     // --- 3. PROJECTILE MATH ---
//     // Note: Again, using turretFieldPos for distance to the lead target
//     double distanceToLead = turretFieldPos.getDistance(leadingTarget);
//     double g = GRAVITY;
//     double h = TARGET_HEIGHT_DIFF;

//     double v2 = v * v;
//     double v4 = v2 * v2;
//     double discriminant = v4 - g * (g * (distanceToLead * distanceToLead) + 2 * h * v2);

//     // SAFETY CHECK: Prevent Math.sqrt(negative_number) returning NaN
//     if (discriminant < 0) {
//         discriminant = 0; 
//         // Note: You might want to add a boolean flag here to tell the shooter 
//         // not to fire, because discriminant < 0 means the target is out of range!
//     }

//     // --- 4. TURRET CONTROL (Azimuth) ---
//     // double turretRotations = turretMotor.getPosition().getValueAsDouble();
//     // double currentTurretAngle = (turretRotations * 360.0) % 360.0;
//     // if (currentTurretAngle < 0) currentTurretAngle += 360.0;

//     // // Calculate angle to the LEADING target using the TURRET'S field position
//     // double angleToLeadRad = Math.atan2(
//     //     leadingTarget.getY() - turretFieldPos.getY(), 
//     //     leadingTarget.getX() - turretFieldPos.getX()
//     // );
    
//     // double turretTargetDeg = Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees();
//     // double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
    
//     // turretMotor.set(Math.abs(turretError) < 3 ? 0 : turretError * KP2);

//     // --- 4. TURRET CONTROL (Azimuth) ---
// // --- 4. TURRET CONTROL (Azimuth) ---
//     double turretRotations = turretMotor.getPosition().getValueAsDouble();
    
//     // 1. Get current turret angle in 0-360 range
//     double currentTurretAngle = (turretRotations * 360.0) % 360.0;
//     if (currentTurretAngle < 0) currentTurretAngle += 360.0;

//     // 2. Calculate absolute field angle to target
//     double angleToLeadDeg = Units.radiansToDegrees(
//         Math.atan2(leadingTarget.getY() - turretFieldPos.getY(), 
//                    leadingTarget.getX() - turretFieldPos.getX())
//     );

//     // 3. Subtract robot rotation to get robot-relative target
//     // 1. Calculate the raw relative angle
// double rawTargetDeg = angleToLeadDeg - robotPos.getRotation().getDegrees();

// // 2. Normalize to 0-360 range
// // This converts -330 into +30, and 390 into +30.
// double turretTargetDeg = rawTargetDeg % 360.0;
// if (turretTargetDeg < 0) {
//     turretTargetDeg += 360.0;
// }

// // 3. Calculate the error using the "shortest path"
// // This prevents the turret from spinning 330 degrees when it only needs to move 30.
// double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
    
//     // 6. Apply output
//     turretMotor.set(Math.abs(turretError) < 3 ? 0 : turretError * KP2);

//     // --- 5. PIVOT CONTROL (Coaxial Compensation) ---

// double targetPitchDegrees;
// double angleRad = Math.atan((v2 - Math.sqrt(discriminant)) / (g * distanceToLead));
// targetPitchDegrees = MathUtil.clamp(Math.toDegrees(angleRad), 2.0, 40.0);

// /**
//  * 1. PITCH COMPONENT
//  * Calculate motor rotations needed to tilt the shooter up/down.
//  */
// double pivotRotationsForPitch = (targetPitchDegrees / 360.0) * PIVOT_GEAR_RATIO;

// /**
//  * 2. COAXIAL COMPENSATION
//  * Every degree the turret mechanism moves, the pivot motor must move
//  * to stay at the same relative pitch. 
//  */
// double turretMotorRotations = beginTurretPose - turretMotor.getPosition().getValueAsDouble();
// double turretMechanismRotations = turretMotorRotations / TURRET_GEAR_RATIO;

// // This calculates how many pivot motor rotations are "lost" or "gained" by the turret spin.
// // We multiply the turret's physical rotations by the pivot's gear ratio to cancel it out.
// double coaxialCompensationRotations = turretMechanismRotations * PIVOT_GEAR_RATIO;

// /**
//  * 3. FINAL SETPOINT
//  * Combine the pitch angle and the turret correction into one target for the motor.
//  */
// double finalPivotTargetRotations = pivotRotationsForPitch + coaxialCompensationRotations;

// // 4. ERROR CALCULATION (Target - Current)
// double currentPivotRotations = pivotMotor.getPosition().getValueAsDouble();
// double pivotErrorRotations = finalPivotTargetRotations - currentPivotRotations;

// // Apply Output using KP1 (0.09)
// double pivotOutput = pivotErrorRotations * KP1;

// // Deadband to prevent jitter (approx 0.5 degrees of error)
// pivotMotor.set(Math.abs(pivotErrorRotations * 360 / PIVOT_GEAR_RATIO) < 0.5 ? 0 : pivotOutput);

//     // --- 6. TELEMETRY ---
//     SmartDashboard.putNumber("Turret/Dist_To_Lead", distanceToLead);
//     // SmartDashboard.putNumber("Turret/Pivot_Error", pivotError);
//     // SmartDashboard.putNumber("Turret/Current_Pitch_Deg", currentPivotDeg);
//     SmartDashboard.putNumber("Turret/Target_Pitch_Deg", targetPitchDegrees);
//     // SmartDashboard.putNumber("Turret/Combined_Target_Pitch_Deg", -combinedPivotTargetDeg);
//     SmartDashboard.putNumber("Turret/Current_Angle_Deg", currentTurretAngle);
//     SmartDashboard.putNumber("Turret/Target_Angle_Deg", turretTargetDeg);
// }
   


// @Override
// public void periodic() {   
//     autoGoal();
// }

// }

