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
import edu.wpi.first.math.controller.PIDController;



public class Turret extends SubsystemBase {

    private final TalonFX turretMotor;
    private final TalonFX pivotMotor;
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;

    // Control Constants
    private final double KP_TURRET = 0.01; 
    private final double TURRET_GEAR_RATIO = 11.0;

    public boolean isUnderTrench = false;

    PIDController pid = new PIDController(KP_TURRET, 0,0);

    
    
    // Physics Constants (Standardized to Feet for the formula)
    private final double TARGET_HEIGHT_DIFF = 1.27;
    private final double SHOOTER_SPEED = 12;

    private final double GRAVITY = 9.81;

    public char alliance;

    private final double SHOOTER_CURRENT_TURRET_FF = -0.35;
    private final double SHOOTER_CURRENT_PIVOT_FF  = 0;

        double targetPitchDegrees;
        double currentTurretAngle;
        double turretTargetDeg;
    
    PositionVoltage request = new PositionVoltage(0);

    // Field Constants (Converted to Meters for WPILib compatibility)
    private final Translation2d redHubMeters = new Translation2d(
        Units.inchesToMeters(469.11), 
        Units.inchesToMeters(158.845)
    );

    private final Translation2d blueHubMeters = new Translation2d(
        Units.inchesToMeters(182.11),
        Units.inchesToMeters(158.845)
    );

    public Turret(CommandSwerveDrivetrain drivetrain, Shooter shooter, char alliance) {
        this.turretMotor = new TalonFX(30);
        this.pivotMotor = new TalonFX(14);
        this.drivetrain = drivetrain;
        this.alliance = alliance;
        this.shooter = shooter;

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO; 
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretConfig.CurrentLimits.StatorCurrentLimit = 45;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.Slot0.kV = 50;
        turretConfig.Slot0.kI = 0.001;


        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Feedback.SensorToMechanismRatio = 52.66;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 45;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        pivotConfig.Slot0.kP = 150;
        pivotConfig.Slot0.kV = 20;
        pivotConfig.Slot0.kD = 3;

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

    public void zeroOnlyPivot()
    {
        double turretRotations = turretMotor.getPosition().getValueAsDouble();
        pivotMotor.setPosition(turretRotations * .2147);
    }

public Command autoTrackingHubCommand(char al) {
        return this.run(
            () -> autoTrackingHub(al)
        );
    }
    

    public void switchTrenchMode(){
        isUnderTrench = !isUnderTrench;
    }

    public Command passingCommand(Pose2d robotPose, char alliance) {
        Translation2d Blue_Left_Target = new Translation2d(2,1);
        Translation2d Blue_Right_Target = new Translation2d(2,7);
        Translation2d Red_Left_Target = new Translation2d(14.535,7);
        Translation2d Red_Right_Target = new Translation2d(14.535,1);

        Translation2d finalTarget; 

        if(alliance == 'R'){
            if(robotPose.getX()>4){
                finalTarget = Red_Left_Target;
            }else{
                finalTarget = Red_Right_Target;
            }
        }else{
            if(robotPose.getX()>4){
                finalTarget = Blue_Right_Target;
            }else{
                finalTarget = Blue_Left_Target;
            }
        }

        return this.run(
            () -> autoTrackingPass(finalTarget)
        );
    }

    public Translation2d passing(Pose2d robotPose, char alliance) {
        Translation2d Blue_Left_Target = new Translation2d(2,1);
        Translation2d Blue_Right_Target = new Translation2d(2,7);
        Translation2d Red_Left_Target = new Translation2d(14.535,7);
        Translation2d Red_Right_Target = new Translation2d(14.535,1);

        Translation2d finalTarget; 

        if(alliance == 'R'){
            if(robotPose.getX()>4){
                finalTarget = Red_Left_Target;
            }else{
                finalTarget = Red_Right_Target;
            }
        }else{
            if(robotPose.getX()>4){
                finalTarget = Blue_Right_Target;
            }else{
                finalTarget = Blue_Left_Target;
            }
        }

        return finalTarget;

        // autoTrackingPass(finalTarget);
        
    }

    public void zeroPivot(){
        PositionVoltage p = new PositionVoltage(0);
        pivotMotor.setControl(p.withPosition(0));
    }

    public Command zeroPivotCommand() {
        return this.runOnce(
            () -> zeroPivot()
        );
    }

    public Command homePivotCommand() {
    return this.run(() -> pivotMotor.set(0.1))
        .withTimeout(0.5)                      
        .andThen(() -> {
            pivotMotor.set(0);             
            zeroOnlyPivot();                       
        }); 
}

    public void zeroTurret(){
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

    public void addToPivot()
    {
        double current = pivotMotor.getPosition().getValueAsDouble();
        pivotMotor.setControl(request.withPosition(current+.01));
    }

     public Command minusPivot(){
        return this.run(
            () -> minusToPivot()
        );
    }

    public void minusToPivot()
    {
        double current = pivotMotor.getPosition().getValueAsDouble();
        pivotMotor.setControl(request.withPosition(current-.01));
    }

    public Command addPivot(){
        return this.run(
            () -> addToPivot()
        );
    }

    public Command switchTurretCommand(){
        return this.runOnce(
            () -> switchTrenchMode()
        );
    }
   
   
public void autoTrackingHub(char a){
    Pose2d robotPos = drivetrain.getState().Pose;
    

    // Get robot-relative speeds and transform them to field-relative
    var robotRelativeSpeeds = drivetrain.getState().Speeds;
    var fieldRelativeSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, 
        robotPos.getRotation()
    );

    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

    double shooterCurrent = shooter.getShooterCurrent();

    // --- 2. PREDICTIVE TARGETING (Motion Compensation) ---    
    // Initial distance estimate to get a baseline flight time

    double distToHub = 0;
    Translation2d desiredHub;

    if (a == 'R') desiredHub = redHubMeters;
    else desiredHub = blueHubMeters;
    
    distToHub = robotPos.getTranslation().getDistance(desiredHub);

    //double v = shooter.getSpeed(); 
    double v = SHOOTER_SPEED;
    double flightTime = distToHub / v; // Rough estimate of time-to-target

    // Calculate lead position: Target = Current - (Velocity * Time)
    Translation2d leadingTarget = new Translation2d(
        desiredHub.getX() - (robotVx * flightTime),
        desiredHub.getY() - (robotVy * flightTime)
    );
    ///    // --- 3. PROJECTILE MATH (Using the Leading Target) ---
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
    turretTargetDeg = MathUtil.inputModulus(
    Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees() + 15,
    0, 360
    );
    double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
    
    double compensatedError = turretError + (shooterCurrent * SHOOTER_CURRENT_TURRET_FF);
    turretMotor.set(Math.abs(compensatedError) < 3 ? 0 : compensatedError * KP_TURRET);




    // --- 5. PIVOT CONTROL (Coaxial Compensation) ---
        
    double angleRad = Math.atan((v2 + Math.sqrt(discriminant)) / (g * distanceToLead));
    // double angleRad = Math.atan((v2 - Math.sqrt(discriminant)) / (g * distanceToLead));

  
    double testAngleDeg = 90-Math.toDegrees(angleRad);

    SmartDashboard.putNumber("Target/Pitch_Angle_Deg", testAngleDeg);
    double angleDeg = MathUtil.clamp(testAngleDeg, 0, 45);
    // targetPitchDegrees = 42-targetPitchDegrees;
    // targetPitchDegrees = angleRad;

      if(isUnderTrench){
        targetPitchDegrees = 0;
    }else{
        targetPitchDegrees = 25;
    }
    
    
    //MathUtil.clamp(targetPitchDegrees, 0, 45);

    //TODO: Play around with 4
    double pitchOnlyRotations = (targetPitchDegrees/360 ) * 0.5  ;//* 1.6; //COME HERE

    double turretTargetRotations = turretTargetDeg/360; // TODO: THIS IS COMPENSATING EARLY
  
    pivotMotor.setControl(request.withPosition(turretRotations * 0.2147 + pitchOnlyRotations + (shooterCurrent * SHOOTER_CURRENT_PIVOT_FF)));

}


@Override
public void periodic() {
   // --- 1. GET ROBOT STATE ---
    // SmartDashboard.putNumber("Turret/Dist_To_Lead", distanceToLead);
    // SmartDashboard.putNumber("Turret/Pivot_Target_Rotations", pitchOnlyRotations);
    SmartDashboard.putNumber("Turret/Current_Pitch_Position", pivotMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Target_Pitch_Deg", targetPitchDegrees );
    
    // SmartDashboard.putNumber("Turret/Combined_Target_Pitch_Deg", combinedPivotTargetDeg);
    SmartDashboard.putNumber("Turret/Current_Turret_Deg", turretMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Target_Turret_Deg", turretTargetDeg);


    //BLUE
    // else if(drivetrain.getState().Pose.getX()>6.5){
    //     passing(drivetrain.getState().Pose, alliance);
    // }

    

    //RED
    // else if(drivetrain.getState().Pose.getX()<6.5){
    //     passing(drivetrain.getState().Pose, alliance);
    // }



        if(this.alliance == 'B'){
            if(drivetrain.getState().Pose.getX()<3.6){
                autoTrackingHub(this.alliance);
            }else if(drivetrain.getState().Pose.getX()>6){
                //logic for passing
                autoTrackingPass(passing(drivetrain.getState().Pose, alliance));
            }
            else{
                pivotMotor.setControl(request.withPosition(0));
                turretMotor.setControl(request.withPosition(0));

                if(pivotMotor.getPosition().getValueAsDouble() <0.1){
                    pivotMotor.setPosition(0);
            
                }

            }
            //sus add an else with command?
        }else{
            if(drivetrain.getState().Pose.getX()>13.5){
                autoTrackingHub(this.alliance);
            }else if(drivetrain.getState().Pose.getX()<10.4){
                //logic for passing
                autoTrackingPass(passing(drivetrain.getState().Pose, alliance));
            }else{
                pivotMotor.setControl(request.withPosition(0));
                turretMotor.setControl(request.withPosition(0));

                if(pivotMotor.getPosition().getValueAsDouble() <0.1){
                    pivotMotor.setPosition(0);
            
                }
            }
            //same here sus add an else with command?
        }
}

public void autoTrackingPass(Translation2d desiredTarget){
    Pose2d robotPos = drivetrain.getState().Pose;
    

    // Get robot-relative speeds and transform them to field-relative
    var robotRelativeSpeeds = drivetrain.getState().Speeds;
    var fieldRelativeSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, 
        robotPos.getRotation()
    );

    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

    double shooterCurrent = shooter.getShooterCurrent();

    // --- 2. PREDICTIVE TARGETING (Motion Compensation) ---
    // Initial distance estimate to get a baseline flight time

    double distToTarget = 0;

    distToTarget = robotPos.getTranslation().getDistance(desiredTarget);

    double v = SHOOTER_SPEED;
    double flightTime = distToTarget / v; // Rough estimate of time-to-target

    // Calculate lead position: Target = Current - (Velocity * Time)
    Translation2d leadingTarget = new Translation2d(
        desiredTarget.getX() - (robotVx * flightTime),
        desiredTarget.getY() - (robotVy * flightTime)
    );

     double turretRotations = turretMotor.getPosition().getValueAsDouble();
    currentTurretAngle = (turretRotations * 360.0) % 360.0;
    if (currentTurretAngle < 0) currentTurretAngle += 360.0;

    // Calculate angle to the LEADING target
    double angleToLeadRad = Math.atan2(
        leadingTarget.getY() - robotPos.getY(), 
        leadingTarget.getX() - robotPos.getX()
    );
    turretTargetDeg = MathUtil.inputModulus(
    Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees() + 15,
    0, 360
    );
    double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
    
    double compensatedError = turretError + (shooterCurrent * SHOOTER_CURRENT_TURRET_FF);
    turretMotor.set(Math.abs(compensatedError) < 3 ? 0 : compensatedError * KP_TURRET);




    // --- 5. PIVOT CONTROL (Coaxial Compensation) ---
        
    //------ double angleRad = Math.atan((v2 + Math.sqrt(discriminant)) / (g * distanceToLead));
    // double angleRad = Math.atan((v2 - Math.sqrt(discriminant)) / (g * distanceToLead));

    if(isUnderTrench){
        targetPitchDegrees = 0;
    }else{
        targetPitchDegrees = 35;
    }
    
    
    //MathUtil.clamp(Math.toDegrees(angleRad), 2.0, 40.0);
    // targetPitchDegrees = 42-targetPitchDegrees;
    targetPitchDegrees = MathUtil.clamp(targetPitchDegrees, 0, 45);

    //TODO: Play around with 4
    double pitchOnlyRotations = (targetPitchDegrees / 360.0 )  ;//* 1.6; //COME HERE

    double turretTargetRotations = turretTargetDeg/360; // TODO: THIS IS COMPENSATING EARLY
  
    pivotMotor.setControl(request.withPosition(turretRotations * 0.2147 + pitchOnlyRotations + (shooterCurrent * SHOOTER_CURRENT_PIVOT_FF)));

//     // --- 3. PROJECTILE MATH (Using the Leading Target) ---
//     double distanceToLead = robotPos.getTranslation().getDistance(leadingTarget);
//     double g = GRAVITY;
//     double h = TARGET_HEIGHT_DIFF;

//     double v2 = v * v;
//     double v4 = v2 * v2;
//     double discriminant = v4 - g * (g * (distanceToLead * distanceToLead) + 2 * h * v2);

//    if(discriminant<0) return; //TODO: Decide if to keep
 

//     // --- 4. TURRET CONTROL (Azimuth) ---
//     double turretRotations = turretMotor.getPosition().getValueAsDouble();
//     currentTurretAngle = (turretRotations * 360.0) % 360.0;
//     if (currentTurretAngle < 0) currentTurretAngle += 360.0;

//     // Calculate angle to the LEADING target
//     double angleToLeadRad = Math.atan2(
//         leadingTarget.getY() - robotPos.getY(), 
//         leadingTarget.getX() - robotPos.getX()
//     );
    
//     turretTargetDeg = MathUtil.inputModulus(Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees(), 0, 360);
//     double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
    
//     double compensatedError = turretError + (shooterCurrent * SHOOTER_CURRENT_TURRET_FF);
//     turretMotor.set(Math.abs(compensatedError) < 3 ? 0 : compensatedError * KP_TURRET);

//     double angleRad = Math.atan((v2 + Math.sqrt(discriminant)) / (g * distanceToLead));
//     targetPitchDegrees = MathUtil.clamp(Math.toDegrees(angleRad), 0.0, 45.0);

//     //TODO: Play around with 4
//     double pitchOnlyRotations = (targetPitchDegrees / 360.0 ) ;// * 4;
  
//     pivotMotor.setControl(request.withPosition(turretRotations * 0.2147 + pitchOnlyRotations + (shooterCurrent * SHOOTER_CURRENT_PIVOT_FF)));
}


}



