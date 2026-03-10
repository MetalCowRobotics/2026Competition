package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final TalonFX turretMotor;
    private final TalonFX pivotMotor;
    private final CommandSwerveDrivetrain drivetrain;

    // Control Constants
    private final double KP = 0.004; 
    private final double GEAR_RATIO = 11.0;
    
    // Physics Constants (Standardized to Feet for the formula)
    private final double TARGET_HEIGHT_DIFF_FT = 5.0;
    private final double SHOOTER_EXIT_VELOCITY_FT_PER_S = 35.0;
    private final double GRAVITY_FT_PER_S2 = 32.17;

    // Field Constants (Converted to Meters for WPILib compatibility)
    private final Translation2d redHubMeters = new Translation2d(
        Units.inchesToMeters(469.11), 
        Units.inchesToMeters(158.845)
    );

    public Turret(CommandSwerveDrivetrain drivetrain) {
        this.turretMotor = new TalonFX(30);
        this.pivotMotor = new TalonFX(14);
        this.drivetrain = drivetrain;

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO; 
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        turretMotor.getConfigurator().apply(config);
        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        // --- 1. DISTANCE CALCULATION (Meters to Feet) ---
        Pose2d robotPosMeters = drivetrain.getState().Pose;
        double distanceToHubMeters = robotPosMeters.getTranslation().getDistance(redHubMeters);
        
        // Convert to feet for the projectile formula
        double d = Units.metersToFeet(distanceToHubMeters);

        // --- 2. PROJECTILE MATH (Degrees) ---
        double v = SHOOTER_EXIT_VELOCITY_FT_PER_S;
        double g = GRAVITY_FT_PER_S2;
        double h = TARGET_HEIGHT_DIFF_FT;

        double v2 = v * v;
        double v4 = v2 * v2;
        double root = v4 - g * (g * (d * d) + 2 * h * v2);

        double targetPivotPitchDeg;
        if (root >= 0 && d > 0.5) {
            // Lower trajectory solution
            double angleRad = Math.atan((v2 - Math.sqrt(root)) / (g * d));
            targetPivotPitchDeg = Math.toDegrees(angleRad);
        } else {
            targetPivotPitchDeg = 45.0; // Failback
        }

        // --- 3. TURRET MOVEMENT (Shortest Path) ---
        double currentTurretRotations = turretMotor.getPosition().getValueAsDouble();
        double currentTurretAngle = (currentTurretRotations * 360.0) % 360.0;
        if (currentTurretAngle < 0) currentTurretAngle += 360.0;

        // atan2 returns radians, we convert to degrees for the error calculation
        double angleToHubRad = Math.atan2(
            redHubMeters.getY() - robotPosMeters.getY(), 
            redHubMeters.getX() - robotPosMeters.getX()
        );
        
        double robotRelativeTarget = Math.toDegrees(angleToHubRad) + robotPosMeters.getRotation().getDegrees();
        double turretError = MathUtil.inputModulus(robotRelativeTarget - currentTurretAngle, -180, 180);
        
        turretMotor.set(Math.abs(turretError) < 1.0 ? 0 : turretError * KP);

        // --- 4. PIVOT MOVEMENT (Coaxial Sync) ---
        double rawPivotRotations = pivotMotor.getPosition().getValueAsDouble();
        
        // Effective Angle = (Actual Pivot Rotations - Turret Rotations) * 360
        double effectivePivotAngle = (rawPivotRotations - currentTurretRotations) * 360.0;
        double pivotError = targetPivotPitchDeg - effectivePivotAngle;

        pivotMotor.set(Math.abs(pivotError) < 1.0 ? 0 : pivotError * KP);

        // Telemetry
        SmartDashboard.putNumber("Distance (Ft)", d);
        SmartDashboard.putNumber("Calculated Pitch (Deg)", targetPivotPitchDeg);
        SmartDashboard.putNumber("Turret Angle (Deg)", currentTurretAngle);

        /*double dx = redHubMeters.getX() - robotPosMeters.getX();
double dy = redHubMeters.getY() - robotPosMeters.getY();

double distance = Math.hypot(dx, dy);

double angleToHubRad = Math.atan2(dy, dx);
double targetFieldAngleDeg = Math.toDegrees(angleToHubRad);

double robotHeadingDeg = robotPosMeters.getRotation().getDegrees();

// Flight time 
double flightTime =
    distance /
    (SHOOTER_EXIT_VELOCITY_METERS_PER_S * Math.cos(angleRad));

// Robot velocity 
var speeds = drivetrain.getState().Speeds;

double robotSpeed =
    Math.hypot(speeds.vxMetersPerSecond,
               speeds.vyMetersPerSecond);

double robotVelocityAngle =
    Math.atan2(speeds.vyMetersPerSecond,
               speeds.vxMetersPerSecond);

// Sideways velocity
double sidewaysVelocity =
    robotSpeed *
    Math.sin(robotVelocityAngle - angleToHubRad);

// Lead compensation
double yawLeadDeg =
    Math.toDegrees(
        Math.atan2(sidewaysVelocity * flightTime, distance)
    );

// Final turret target
double robotRelativeTarget =
    targetFieldAngleDeg
    - robotHeadingDeg
    + yawLeadDeg; */
    }
}





// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Turret extends SubsystemBase {

//     private final TalonFX turretMotor;
//     private final TalonFX pivotMotor;
//     private final CommandSwerveDrivetrain drivetrain;

//     private double targetPivotPitchDeg = 25.0; 
//     private final double KP = 0.004; 
//     private final double GEAR_RATIO = 11.0;

//     public Turret(CommandSwerveDrivetrain drivetrain) {
//         this.turretMotor = new TalonFX(30);
//         this.pivotMotor = new TalonFX(14);
//         this.drivetrain = drivetrain;

//         configureMotors();
//     }

//     private void configureMotors() {
//         TalonFXConfiguration config = new TalonFXConfiguration();
//         config.Feedback.SensorToMechanismRatio = GEAR_RATIO; 
//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//         turretMotor.getConfigurator().apply(config);
//         pivotMotor.getConfigurator().apply(config);
//     }

//     @Override
//     public void periodic() {
//         // --- 1. TURRET LOGIC ---
//         double currentTurretRotations = turretMotor.getPosition().getValueAsDouble();
//         double currentTurretAngle = (currentTurretRotations * 360.0) % 360.0;
//         if (currentTurretAngle < 0) currentTurretAngle += 360.0;

//         Translation2d redHub = new Translation2d(469.11, 158.845);
//         Pose2d robotPos = drivetrain.getState().Pose;

//         double angleToHubRad = Math.atan2(redHub.getY() - robotPos.getY(), redHub.getX() - robotPos.getX());
//         double robotRelativeTarget = Math.toDegrees(angleToHubRad) + robotPos.getRotation().getDegrees();
        
//         // Shortest path error for continuous rotation
//         double turretError = MathUtil.inputModulus(robotRelativeTarget - currentTurretAngle, -180, 180);

//         if (Math.abs(turretError) < 1.0) {
//             turretMotor.set(0);
//         } else {
//             turretMotor.set(turretError * KP);
//         }

//         // --- 2. PIVOT LOGIC (COAXIAL POWER-BASED) ---
        
//         // Calculate the "Effective" Pivot Angle:
//         // Because it's coaxial, we subtract the turret's rotation to see where the pivot 
//         // is pointing relative to the turret plate.
//         double rawPivotRotations = pivotMotor.getPosition().getValueAsDouble();
//         double effectivePivotAngle = (rawPivotRotations - currentTurretRotations) * 360.0;

//         // Calculate Pivot Error based on your 25 degree target
//         double pivotError = targetPivotPitchDeg - effectivePivotAngle;

//         // Apply the same power logic as the turret
//         if (Math.abs(pivotError) < 1.0) {
//             pivotMotor.set(0);
//         } else {
//             pivotMotor.set(pivotError * KP);
//         }

//         // Telemetry
//         SmartDashboard.putNumber("Turret Angle", currentTurretAngle);
//         SmartDashboard.putNumber("Pivot Angle", effectivePivotAngle);
//         SmartDashboard.putNumber("Pivot Error", pivotError);
//     }

//     public void setPivotPitch(double degrees) {
//         this.targetPivotPitchDeg = degrees;
//     }
// }
// package frc.robot.subsystems;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Turret extends SubsystemBase {

//     private final TalonFX turretMotor;
//     private final TalonFX pivotMotor;
//     private final CommandSwerveDrivetrain drivetrain;

//     // Control Request objects for Phoenix 6
//     private final PositionVoltage pivotPositionRequest = new PositionVoltage(0);

//     private double targetPivotPitchDeg = 0; // The "Hood" angle relative to the turret
//     private final double KP = 0.004; 

//     public Turret(CommandSwerveDrivetrain drivetrain) {
//         turretMotor = new TalonFX(30);
//         pivotMotor = new TalonFX(14);
//         this.drivetrain = drivetrain;

//         configureMotors();
//     }

//     public void configureMotors() {
//         TalonFXConfiguration config = new TalonFXConfiguration();
        
//         // Setup for 11:1 ratio
//         config.Feedback.SensorToMechanismRatio = 11; 
//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//         // PID for the Pivot Motor to follow the Turret precisely
//         // You may need to tune these values for your specific weight/friction
//         config.Slot0.kP = 0.004; // Example P gain for Position Control
//         config.Slot0.kI = 0.0;
//         config.Slot0.kD = 0.1;

//         turretMotor.getConfigurator().apply(config);
//         pivotMotor.getConfigurator().apply(config); // Same config since ratios match
//     }

//     @Override
//     public void periodic() {
//         // 1. Existing Turret Tracking Logic
//         double currentTurretAngle = turretMotor.getPosition().getValueAsDouble() * 360.0;
//         Translation2d redHub = new Translation2d(469.11, 158.845);
//         Pose2d robotPos = drivetrain.getState().Pose;

//         double angleToHubRad = Math.atan2(redHub.getY() - robotPos.getY(), redHub.getX() - robotPos.getX());
//         double robotRelativeTarget = Math.toDegrees(angleToHubRad) + robotPos.getRotation().getDegrees();
//         double error = MathUtil.inputModulus(robotRelativeTarget - currentTurretAngle, -180, 180);

//         // Continue using your power-based turret tracking
//         if (Math.abs(error) < 1.0) {
//             turretMotor.set(0);
//         } else {
//             turretMotor.set(error * KP);
//         }
//          if (Math.abs(error) < 1.0) {
//             pivotMotor.set(0);
//         } else {
//             pivotMotor.set(error * KP);
//         }
//         // 2. COAXIAL POSITION SYNC
//         // Get the actual position of the turret in rotations
//         double turretRotations = turretMotor.getPosition().getValueAsDouble();

//         // Telemetry
//         SmartDashboard.putNumber("Turret Motor Pos", turretRotations);
//     }

//     public void setPivotPitch(double degrees) {
//         this.targetPivotPitchDeg = degrees;
//     }
// }
// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Turret extends SubsystemBase {

//     private final TalonFX turretMotor;
//     private final TalonFX pivotMotor; // CAN ID 19
//     private final CommandSwerveDrivetrain drivetrain;

//     // Turret Constants
//     private final double TURRET_GEAR_RATIO = 11.0;
//     private final double TURRET_KP = 0.003;
//     private final double TURRET_TOLERANCE_DEG = 2.0;

//     // Pivot Constants
//     private final double PIVOT_GEAR_RATIO = 39.1; // Update this to your actual hood ratio
//     private final double PIVOT_KP = 0.001;         // Tune this for hood responsiveness
//     private double targetPivotAngleDeg = 0;

//     public Turret(CommandSwerveDrivetrain drivetrain) {
//         turretMotor = new TalonFX(30);
//         pivotMotor = new TalonFX(19);
//         this.drivetrain = drivetrain;

//         configureMotors();
//     }

//     public void configureMotors() {
//         // --- Turret Config ---
//         TalonFXConfiguration turretConfig = new TalonFXConfiguration();
//         turretConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO; 
//         turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//         // --- Pivot Config ---
//         TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
//         pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
//         pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         // Adjust inversion based on your mechanical assembly
//         pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 

//         applyConfig(turretMotor, turretConfig);
//         applyConfig(pivotMotor, pivotConfig);
//     }

//     private void applyConfig(TalonFX motor, TalonFXConfiguration config) {
//         StatusCode status = StatusCode.StatusCodeNotInitialized;
//         for (int i = 0; i < 5; i++) {
//             status = motor.getConfigurator().apply(config);
//             if (status.isOK()) break;
//         }
//     }

//     public double getCurrentTurretAngleDeg() {
//         return turretMotor.getPosition().getValueAsDouble() * 360.0;
//     }

//     public double getCurrentPivotAngleDeg() {
//         return pivotMotor.getPosition().getValueAsDouble() * 360.0;
//     }

//     public void setPivotTarget(double degrees) {
//         this.targetPivotAngleDeg = degrees;
//     }

//     @Override
//     public void periodic() {
//         // --- 1. TURRET LOGIC (Existing) ---
//         double currentTurretAngle = getCurrentTurretAngleDeg();
//         Translation2d redHub = new Translation2d(469.11, 158.845);
//         Pose2d robotPos = drivetrain.getState().Pose;

//         double angleToHubRad = Math.atan2(redHub.getY() - robotPos.getY(), redHub.getX() - robotPos.getX());
//         double robotRelativeTarget = Math.toDegrees(angleToHubRad) + robotPos.getRotation().getDegrees();
//         double turretError = MathUtil.inputModulus(robotRelativeTarget - currentTurretAngle, -180, 180);

//         double turretOutput = (Math.abs(turretError) < TURRET_TOLERANCE_DEG) ? 0 : turretError * TURRET_KP;
//         turretMotor.set(turretOutput);

//         // --- 2. PIVOT COAXIAL LOGIC ---
//         double currentPivotAngle = getCurrentPivotAngleDeg();
//         double pivotError = targetPivotAngleDeg - currentPivotAngle;
        
//         /* * COAXIAL COMPENSATION:
//          * We add the turret's current velocity (or output) to the pivot's output.
//          * If the turret moves, the pivot "fights" that movement to stay still 
//          * relative to the turret base.
//          */
//         double pivotPIDOutput = pivotError * PIVOT_KP;
//         double coaxialCompensation = -(turretOutput);
        
//         // Final pivot power = move to target + cancel out turret movement
//         pivotMotor.set(pivotPIDOutput + coaxialCompensation);

//         // Telemetry
//         SmartDashboard.putNumber("Turret Angle", currentTurretAngle);
//         SmartDashboard.putNumber("Pivot Angle", currentPivotAngle);
//         SmartDashboard.putNumber("Pivot Target", targetPivotAngleDeg);
//     }
// }

// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Turret extends SubsystemBase {

//     private final TalonFX turretMotor;
//     private final CommandSwerveDrivetrain drivetrain;

//     public double targetAngleDeg = 0;
//     private final double TOLERANCE_DEG = 2.0;
//     private final double KP = 0.003; // Matches your current speed multiplier

//     public Turret(CommandSwerveDrivetrain drivetrain) {
//         turretMotor = new TalonFX(30);
//         this.drivetrain = drivetrain;

//         configureMotors();
//     }

//     public void configureMotors() {
//         TalonFXConfiguration config = new TalonFXConfiguration();

//         // 11:1 reduction - FeedbackConfigs handles the gear ratio math
//         config.Feedback.SensorToMechanismRatio = 11; 
//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//         StatusCode status = StatusCode.StatusCodeNotInitialized;
//         for (int i = 0; i < 5; i++) {
//             status = turretMotor.getConfigurator().apply(config);
//             if (status.isOK()) break;
//         }

//         if (!status.isOK()) {
//             System.out.println("Turret motor config failed: " + status.toString());
//         }
//     }

//     /**
//      * Gets the current turret angle in degrees. 
//      * Since SensorToMechanismRatio is 11, getPosition() returns mechanism rotations.
//      */
//     public double getCurrentAngleDeg() {
//         return turretMotor.getPosition().getValueAsDouble() * 360.0;
//     }

//     public void setSpeed(double targetSpeed) {
//         turretMotor.set(targetSpeed);
//     }

//     public Command stopCommand() {
//         return this.runOnce(() -> turretMotor.set(0));
//     }

//     @Override
//     public void periodic() {
//         double currentAngle = getCurrentAngleDeg();
        
//         // Target Hub Location (Red Hub example)
//         Translation2d redHub = new Translation2d(469.11, 158.845);
//         Pose2d robotPos = drivetrain.getState().Pose;

//         // 1. Calculate the angle from the robot to the hub on the field
//         // Math.atan2(y, x) returns the angle in radians
//         double angleToHubRad = Math.atan2(
//             redHub.getY() - robotPos.getY(),
//             redHub.getX() - robotPos.getX()
//         );

//         // 2. Convert to degrees and subtract robot rotation to get "Robot Relative" target
//         double fieldTargetDeg = Math.toDegrees(angleToHubRad);
//         double robotRelativeTarget = fieldTargetDeg + robotPos.getRotation().getDegrees();

//         // 3. THE CONTINUOUS FIX: Calculate the shortest distance to the target
//         // This prevents the turret from spinning 350 degrees to reach -10 from +10.
//         double error = MathUtil.inputModulus(robotRelativeTarget - currentAngle, -180, 180);

//         // 4. Apply Tolerance and Motor Output
//         if (Math.abs(error) < TOLERANCE_DEG) {
//             turretMotor.set(0); // Within 2 degrees, stop moving
//         } else {
//             // Maintains your existing speed scaling
//             turretMotor.set(error * KP); 
//         }

//         // Telemetry
//         SmartDashboard.putNumber("Turret Angle (deg)", currentAngle);
//         SmartDashboard.putNumber("Turret Target (deg)", robotRelativeTarget);
//         SmartDashboard.putNumber("Turret Error (deg)", error);
//         SmartDashboard.putBoolean("Turret At Target", Math.abs(error) < TOLERANCE_DEG);
//     }
// }
    
    // public boolean atTarget() {
    //     return pidController.atSetpoint();
    // }
    // public double normalizeAngleDeg(double angle)
    // {
    //     return MathUtil.inputModulus(angle, -180.0, 180.0);
    // }


 



    // public Command puttingPosition(double targetPosition){
    //      return this.run(
    //         // When the command starts, run the intake
    //         () -> setSpeed(PIDController.calculate(turretMotor.getPosition().getValueAsDouble(), targetPosition;
    //     );
    // }

    // // motor.set(pid.calculate(encoder.getDistance(), setpoint));

    //  public Command (double targetSpeed){
    //            return this.run(
    //         // When the command starts, run the intake
    //         () -> setSpeed(targetSpeed)
    //     );
    // }
 

  


    // public Command goToAngle(int angle) {
    //     return this.runOnce(
    //         // When the command starts, run the intake
    //         () -> setTargetPosition(angle)
    //     );
    // }

    // public Command goToAngle(Pose2d pose, char cha ) {
    //     return this.runOnce(
    //         // When the command starts, run the intake
    //         () -> setTargetPosition(getYawAngle(pose, cha))
    //     );
    // }

    // public Command goToAngle(Pose2d pose, ChassisSpeeds csp, char cha ) {
    //     return this.runOnce(
    //         // When the command starts, run the intake
    //         () -> setTargetPosition(getYawAngle(pose, csp, cha))
    //     );
    // }

    // public double getYawAngle(Pose2d pose, char alliance)
    // {
    //     double angleRad = calculateTrajectory(pose, alliance);
    //     double angleDeg = Math.toDegrees(angleRad);
    //     return normalizeAngleDeg(angleDeg);
    // }

    // public double getPitchAngle(Pose2d pose, ChassisSpeeds speeds, char alliance)
    // {
    //     params = calculateTrajectory(pose, speeds, alliance);
    //     return Math.toDegrees(params.pitchAngle);
    // }

    // public double getYawAngle(Pose2d pose, ChassisSpeeds cSpeeds , char all)
    // {
    //     params = calculateTrajectory(pose, cSpeeds, all);
    //     return Math.toDegrees(params.yawAngle);
    // }


    // public double getTargetAngleDeg() 
    // {
    //     double angle = turretMotor.getPosition().getValueAsDouble() * 360.0;
    //     return normalizeAngleDeg(angle);
    // }


   


//     public double calculateTrajectory(Pose2d robotPose, char alliance) {

//         /* ================= Hub Poses ================= */

//         // Blue Alliance Hub: (4.625, 4.035)
//         Pose2d blueHub = new Pose2d(
//                 4.625,
//                 4.035,
//                 new Rotation2d()
//         );

//         // Red Alliance Hub: (11.92, 4.035)
//         Pose2d redHub = new Pose2d(
//                 11.92,
//                 4.035,
//                 new Rotation2d()
//         );

//         Pose2d hub = (alliance == 'R') ? redHub : blueHub;

//         /* ================= Distance to Target ================= */

//         double dx = hub.getX() - robotPose.getX();
//         double dy = hub.getY() - robotPose.getY();

//         /* ================= Yaw ================= */

//         double angle = Math.atan2(dy, dx);

//         /* ================= Projectile Math ================= */

//         return angle;
//     }
// }

//     // public ShootingParams calculateTrajectory(
//     //     Pose2d robotPose,
//     //     ChassisSpeeds robotVelocity,
//     //     char alliance) {

//     // ShootingParams tparams = new ShootingParams();

//     /* ================= Constants ================= */

//     final double g = 9.81;
//     final double shooterHeight = 0.9;   // meters
//     final double targetHeight = 2.64;   // meters
//     final double fixedVelocity = 12.0;  // m/s (example fixed shooter velocity)

//     /* ================= Hub Pose ================= */

//     Pose2d blueHub = new Pose2d(4.625, 4.035, new Rotation2d());
//     Pose2d redHub  = new Pose2d(11.92,  4.035, new Rotation2d());
//     Pose2d hub = (alliance == 'R') ? redHub : blueHub;

//     /* ================= Distance ================= */

//     double dx = hub.getX() - robotPose.getX();
//     double dy = hub.getY() - robotPose.getY();
//     double distance = Math.hypot(dx, dy);

//     double heightDifference = targetHeight - shooterHeight;

//     /* ================= Solve For Hood Angle ================= */

//     double v2 = fixedVelocity * fixedVelocity;

//     double discriminant =
//             v2 * v2
//             - g * (g * distance * distance
//                    + 2 * heightDifference * v2);

//     if (discriminant < 0) {
//         // Shot not possible at this velocity
//         return tparams;
//     }

//     double sqrt = Math.sqrt(discriminant);

//     // High arc solution (use +)
//     double numerator = v2 + sqrt;
//     double denominator = g * distance;

//     double hoodAngle = Math.atan(numerator / denominator);

//     tparams.pitchAngle = hoodAngle;

//     /* ================= Flight Time ================= */

//     double flightTime =
//             distance / (fixedVelocity * Math.cos(hoodAngle));

//     /* ================= Robot Motion Compensation ================= */

//     double robotSpeed = Math.hypot(
//             robotVelocity.vxMetersPerSecond,
//             robotVelocity.vyMetersPerSecond);

//     double robotVelocityAngle = Math.atan2(
//             robotVelocity.vyMetersPerSecond,
//             robotVelocity.vxMetersPerSecond);

//     double robotToTargetAngle = Math.atan2(dy, dx);

//     double sidewaysVelocity =
//             robotSpeed *
//             Math.sin(robotVelocityAngle - robotToTargetAngle);

//     double yawAngle =
//             Math.atan2(sidewaysVelocity * flightTime, distance);

//     tparams.yawAngle = yawAngle;

//     SmartDashboard.putNumber("Pitch Angle", Math.toDegrees(tparams.pitchAngle));
//     SmartDashboard.putNumber("Yaw Angle", Math.toDegrees(tparams.yawAngle));

//     return tparams;
// }
// }
//     public double calculateFlightTime(
//         double velocity,       // m/s
//         double angleRadians,   // radians
//         double shooterHeight,  // meters
//         double targetHeight    // meters
// ) {
//     double g = 9.81;

//     double verticalVelocity = velocity * Math.sin(angleRadians);
//     double heightDifference = targetHeight - shooterHeight;

//     double discriminant =
//             verticalVelocity * verticalVelocity
//             - 2 * g * heightDifference;

//     if (discriminant < 0) {
//         // Target unreachable
//         return -1;
//     }

//     double sqrt = Math.sqrt(discriminant);

//     // Use larger root
//     double time =
//             (verticalVelocity + sqrt) / g;

//     return time;
// }

//     public ShootingParams calculateTrajectory(Pose2d robotPose,ChassisSpeeds robotVelocity, char alliance, double time) {

    // double gravity = 10.0;          // m/s^2 (close enough to 9.8)
    // double apexFraction = 0.7;      // fraction of distance where apex occurs
    // double apexHeight = 2.25;       // meters
    // double shooterHeight = 0;
    // double targetHeight = 0;
    // double shooterVelocity = 0;

    // /* ================= Hub Poses ================= */

    // // Blue Alliance Hub: (4.625, 4.035)
    // Pose2d blueHub = new Pose2d(
    //         4.625,
    //         4.035,
    //         new Rotation2d()
    // );

    // // Red Alliance Hub: (11.92, 4.035)
    // Pose2d redHub = new Pose2d(
    //         11.92,
    //         4.035,
    //         new Rotation2d()
    // );

    // Pose2d hub = (alliance == 'R') ? redHub : blueHub;

    // /* ================= Distance to Target ================= */

    // double dx = hub.getX() - robotPose.getX();
    // double dy = hub.getY() - robotPose.getY();

    // double trajectoryDistance = Math.sqrt(dx * dx + dy * dy);
    // double totalDistance = Math.hypot(dx, dy);

    // /* ================= Projectile Math ================= */

    // // t = sqrt(2h / g)
    // double timeUntilApex = Math.sqrt((2.0 * apexHeight) / gravity);

    // double distanceUntilApex = trajectoryDistance * apexFraction;

    // double horizontalVelocity = distanceUntilApex / timeUntilApex;
    // double verticalVelocity = gravity * timeUntilApex;

    // /* ================= Pitch ================= */

    // double pitchAngle = Math.atan2(verticalVelocity, horizontalVelocity);

    // /* ================= Robot Velocity ================= */

    // double robotSpeed = Math.hypot(robotVelocity.vxMetersPerSecond,
    //                                robotVelocity.vyMetersPerSecond);

    // double robotVelocityAngle =
    //         Math.atan2(robotVelocity.vyMetersPerSecond,
    //                    robotVelocity.vxMetersPerSecond);

    // double robotToTargetAngle = Math.atan2(dy, dx);

    // /* ================= Sideways Velocity ================= */

    // double vrs =
    //         robotSpeed *
    //         Math.sin(robotVelocityAngle - robotToTargetAngle);

    // /* ================= Yaw Compensation ================= */

    // double verticalV = shooterVelocity * Math.sin(Math.toRadians(pitchAngle));
    // double heightDifference = targetHeight - shooterHeight;

    // double discriminant =
    //         verticalV * verticalV
    //         - 2 * gravity * heightDifference;

    
    // double sqrt = Math.sqrt(discriminant);

    // // Use larger root
    // double flightTime =
    //         (verticalV + sqrt) / gravity;

    // params.yawAngle =
    //         Math.atan2(vrs * flightTime, totalDistance);

       
    // /* ================= Telemetry ================= */

    // SmartDashboard.putNumber("Yaw Angle (rad)", params.yawAngle);
    // SmartDashboard.putNumber("Pitch Angle (rad)", params.pitchAngle);

    // return params;
    // }
