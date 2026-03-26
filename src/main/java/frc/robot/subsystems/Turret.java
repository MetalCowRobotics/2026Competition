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
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;
import edu.wpi.first.math.controller.PIDController;



public class Turret extends SubsystemBase {

    private final TalonFX turretMotor;
    private final TalonFX pivotMotor;
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final PivotLookup pivotLookup;

    // Control Constants


    public boolean isUnderTrench = false;
    public boolean isManualZeroPivot = false;

    PIDController pid = new PIDController(TurretConstants.KP_TURRET, 0,0);

    
    
    // Physics Constants (Standardized to Feet for the formula)
    private final double TARGET_HEIGHT_DIFF = 1.27;
    private final double SHOOTER_SPEED = 12;

    private final double GRAVITY = 9.81;

    public char alliance;

    private final double SHOOTER_CURRENT_TURRET_FF = -0.2;
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
        pivotLookup = new PivotLookup();

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.Feedback.SensorToMechanismRatio = TurretConstants.TURRET_GEAR_RATIO; 
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
    return this.run(() -> pivotMotor.set(-0.1))
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
        Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees() + TurretConstants.TURRET_OFFSET_DEG,
        0, 360
        );

        double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
        
        double compensatedError = turretError + (shooterCurrent * SHOOTER_CURRENT_TURRET_FF);
        turretMotor.set(Math.abs(compensatedError) < 3 ? 0 : compensatedError * TurretConstants.KP_TURRET);


        // --- 5. PIVOT CONTROL (Using Lookup Table) ---

        // 1. Get distance in meters (already calculated earlier)
        double distanceMeters = distToHub;

        if(isUnderTrench){
            targetPitchDegrees = 0;
        }else if(isManualZeroPivot){
            targetPitchDegrees = -10;
        }else{
            targetPitchDegrees = pivotLookup.calculateHoodAngle(distanceMeters);
        }
         
        targetPitchDegrees = MathUtil.clamp(targetPitchDegrees, 0, 45);

        double pitchOnlyRotations = targetPitchDegrees / 360.0;

        double turretCurrentRotations = turretMotor.getPosition().getValueAsDouble();

        pivotMotor.setControl(request.withPosition(pitchOnlyRotations + (turretCurrentRotations * 0.2088)));

        if(targetPitchDegrees == -10 && Math.abs((pitchOnlyRotations + (turretCurrentRotations * 0.2088)) - pivotMotor.getPosition().getValueAsDouble()) < 0.003){
            pivotMotor.setPosition(turretMotor.getPosition().getValueAsDouble());
            isManualZeroPivot = false;
        }
    }

    public void manualZeroPivot(){
        isManualZeroPivot = true;
    }

     public void manualZeroPivotFalse(){
        isManualZeroPivot = false;
    }



    public Command manualZeroPivotCommand(){
        return this.startEnd(
            () -> manualZeroPivot(),
            () -> manualZeroPivotFalse()
        );
    }


@Override
public void periodic() {
   // --- Telemetry ---
    SmartDashboard.putNumber("Turret/Current_Pitch_Position", pivotMotor.getPosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Turret/Target_Pitch_Deg", targetPitchDegrees );
    SmartDashboard.putBoolean("Manual Zero Pivot", isManualZeroPivot);

    
    SmartDashboard.putNumber("Turret/Current_Turret_Deg", turretMotor.getPosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Turret/Target_Turret_Deg", turretTargetDeg);

    //autoTrackingHubCommand(alliance);


    //BLUE
    // else if(drivetrain.getState().Pose.getX()>6.5){
    //     passing(drivetrain.getState().Pose, alliance);
    // }

    

    //RED
    // else if(drivetrain.getState().Pose.getX()<6.5){
    //     passing(drivetrain.getState().Pose, alliance);
    // }



        // if(this.alliance == 'B'){
        //     if(drivetrain.getState().Pose.getX()<3.6){
        //         autoTrackingHub(this.alliance);
        //     }else if(drivetrain.getState().Pose.getX()>6){
        //         //logic for passing
        //         autoTrackingPass(passing(drivetrain.getState().Pose, alliance));
        //     }
        //     else{
        //         pivotMotor.setControl(request.withPosition(0));
        //         // turretMotor.setControl(request.withPosition(0));

        //         if(pivotMotor.getPosition().getValueAsDouble() <0.1){
        //             pivotMotor.setPosition(0);
            
        //         }

        //     }
        //     //sus add an else with command?
        // }else{
        //     if(drivetrain.getState().Pose.getX()>13.5){
        //         autoTrackingHub(this.alliance);
        //     }else if(drivetrain.getState().Pose.getX()<10.4){
        //         //logic for passing
        //         autoTrackingPass(passing(drivetrain.getState().Pose, alliance));
        //     }else{
        //         pivotMotor.setControl(request.withPosition(0));
        //         //turretMotor.setControl(request.withPosition(0));

        //         if(pivotMotor.getPosition().getValueAsDouble() <0.1){
        //             pivotMotor.setPosition(0);
            
        //         }
        //     }
        //     //same here sus add an else with command?
        // }
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
    Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees() + TurretConstants.TURRET_OFFSET_DEG,
    0, 360
    );
    double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);
    
    double compensatedError = turretError + (shooterCurrent * SHOOTER_CURRENT_TURRET_FF);
    //turretMotor.set(Math.abs(compensatedError) < 3 ? 0 : compensatedError * KP_TURRET);




    // --- 5. PIVOT CONTROL (Coaxial Compensation) ---
    //TODO: Play around with 4
    double pitchOnlyRotations = (targetPitchDegrees / 360.0 )  ;//* 1.6; //COME HERE
  
    pivotMotor.setControl(request.withPosition(turretRotations * 0.2147 + pitchOnlyRotations + (shooterCurrent * SHOOTER_CURRENT_PIVOT_FF)));
    }
}



