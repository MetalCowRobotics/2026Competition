package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class Turret extends SubsystemBase {

    private final TalonFX turretMotor;
    private final TalonFX pivotMotor;
    private final CommandSwerveDrivetrain drivetrain;

    // Control Constants
    private final double KP_TURRET = 0.01; 
    private final double TURRET_GEAR_RATIO = 11.0;
    private final double PIVOT_GEAR_RATIO = 50.28571428571429; // Your custom multiplier

    PIDController pid = new PIDController(KP_TURRET, 0,0);
    
    
    // Physics Constants (Standardized to Feet for the formula)
    private final double TARGET_HEIGHT_DIFF = 1.27;
    private final double SHOOTER_SPEED = 10.77;
    private final double GRAVITY = 9.81;

        double targetPitchDegrees;
        double currentTurretAngle;
        double turretTargetDeg;
    

    private final SlewRateLimiter pivotRamp = new SlewRateLimiter(0.5);
    
    PositionVoltage request = new PositionVoltage(0);

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
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO; 
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretConfig.CurrentLimits.StatorCurrentLimit = 45;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.Slot0.kV = 250;
        turretConfig.Slot0.kI = 0.001;


        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Feedback.SensorToMechanismRatio = 52.66;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 45;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        pivotConfig.Slot0.kP = 45;
        pivotConfig.Slot0.kV = 250;

        zeroMotors();
      
      
        turretMotor.getConfigurator().apply(turretConfig);
        pivotMotor.getConfigurator().apply(pivotConfig);


    }
    
    /**
     * Manually resets encoder positions to zero. 
     * Use this when the turret and pivot are physically at their "home" positions.
     */
    public void zeroMotors() {
        turretMotor.setPosition(0);
        pivotMotor.setPosition(0);
    }

    public Command autoTrackingCommand() {
        return this.run(
            () -> autoTracking()
        );
    }

    private void zeroPivot(){
        PositionVoltage p = new PositionVoltage(0);
        pivotMotor.setControl(p.withPosition(0));
    }

    public Command zeroPivotCommand() {
        return this.run(
            () -> zeroPivot()
        );
    }

    private void zeroTurret(){
        PositionVoltage p = new PositionVoltage(0);
        turretMotor.setControl(p.withPosition(0));
    }

    public Command zeroTurretCommand() {
        return this.run(
            () -> zeroTurret()
        );
    }

    public Command lowerPivot()
    {
        return this.run(
            () -> pivotMotor.set(-0.1)
        );
    }

    public Command stopPivot(){
        return this.run(
            () -> pivotMotor.set(0)
        );
    }
   
    private void autoTracking(){
    Pose2d robotPos = drivetrain.getState().Pose;
    

    // Get robot-relative speeds and transform them to field-relative
    var robotRelativeSpeeds = drivetrain.getState().Speeds;
    var fieldRelativeSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, 
        robotPos.getRotation()
    );

    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;


    // --- 2. PREDICTIVE TARGETING (Motion Compensation) ---
    // Initial distance estimate to get a baseline flight time
    double distToHub = robotPos.getTranslation().getDistance(redHubMeters);
    double v = SHOOTER_SPEED;
    double flightTime = distToHub / v; // Rough estimate of time-to-target

    // Calculate lead position: Target = Current - (Velocity * Time)
    Translation2d leadingTarget = new Translation2d(
        redHubMeters.getX() - (robotVx * flightTime),
        redHubMeters.getY() - (robotVy * flightTime)
    );

    // --- 3. PROJECTILE MATH (Using the Leading Target) ---
    double distanceToLead = robotPos.getTranslation().getDistance(leadingTarget);
    double g = GRAVITY;
    double h = TARGET_HEIGHT_DIFF;

    double v2 = v * v;
    double v4 = v2 * v2;
    double discriminant = v4 - g * (g * (distanceToLead * distanceToLead) + 2 * h * v2);

   if(discriminant<0) return; //TODO: Decide if to keep
 

    // --- 4. TURRET CONTROL (Azimuth) ---
    double turretRotations = turretMotor.getPosition().getValueAsDouble();
    currentTurretAngle = (turretRotations * 360.0) % 360.0;
    if (currentTurretAngle < 0) currentTurretAngle += 360.0;

    // Calculate angle to the LEADING target
    double angleToLeadRad = Math.atan2(
        leadingTarget.getY() - robotPos.getY(), 
        leadingTarget.getX() - robotPos.getX()
    );
    
    turretTargetDeg = Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees();
    double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
    
    turretMotor.set(Math.abs(turretError) < 3 ? 0 : turretError * KP_TURRET);

    // --- 5. PIVOT CONTROL (Coaxial Compensation) ---
        
    double angleRad = Math.atan((v2 - Math.sqrt(discriminant)) / (g * distanceToLead));
    targetPitchDegrees = MathUtil.clamp(Math.toDegrees(angleRad), 2.0, 40.0);
    targetPitchDegrees = 42-targetPitchDegrees;
    targetPitchDegrees = MathUtil.clamp(targetPitchDegrees, 1, 12);

    //TODO: Play around with 4
    double pitchOnlyRotations = (targetPitchDegrees / 360.0 )  * 4;
  
    pivotMotor.setControl(request.withPosition(turretRotations*0.2147+pitchOnlyRotations));
}


@Override
public void periodic() {
   // --- TELEMETRY ---

    SmartDashboard.putNumber("Turret/Current_Pitch_Position", pivotMotor.getPosition().getValueAsDouble() * 360);
    SmartDashboard.putNumber("Turret/Target_Pitch_Deg", targetPitchDegrees );
    SmartDashboard.putNumber("Turret/Current_Turret_Deg", currentTurretAngle);
    SmartDashboard.putNumber("Turret/Target_Turret_Deg", turretTargetDeg);
}


}
