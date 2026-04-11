package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import edu.wpi.first.wpilibj.DigitalInput;

// Initialize a DigitalInput on DIO port 0

// Get the current value: true if the circuit is open (High), false if closed (Low)



public class Turret extends SubsystemBase {

    private final TalonFX turretMotor;
    private final TalonFX pivotMotor;
    private final CommandSwerveDrivetrain drivetrain;
    private final PivotLookup pivotLookup;


    private final double SHOOTER_SPEED = 7;

    public char alliance;

    double targetPitchDegrees;
    double currentTurretAngle;
    double turretTargetDeg;
    double turretpivotTargetRotations;
    double turretTarget;
    double distToHub;
    DigitalInput limitSwitch = new DigitalInput(1);
    boolean isTriggered = limitSwitch.get();
    PositionVoltage request = new PositionVoltage(0);
   
    public boolean getSwitch(){
        return limitSwitch.get();
    }

    // Field Constants (Converted to Meters for WPILib compatibility)
    private final Translation2d redHubMeters = new Translation2d(
            Units.inchesToMeters(469.11),
            Units.inchesToMeters(158.845));

    private final Translation2d blueHubMeters = new Translation2d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(158.845));

    public Turret(CommandSwerveDrivetrain drivetrain, Shooter shooter, char alliance) {
        this.turretMotor = new TalonFX(14);// 30
        this.pivotMotor = new TalonFX(30);// 14
        this.drivetrain = drivetrain;
        this.alliance = alliance;
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
        // turretConfig.Slot0.kP = 150;
        // turretConfig.Slot0.kV = 0;
        // turretConfig.Slot0.kI = 0.001;

        turretConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
        turretConfig.Slot0.kP = 150;
        turretConfig.Slot0.kV = 0;
        turretConfig.Slot0.kD = 2.5;
        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turretConfig.ClosedLoopGeneral.ContinuousWrap = true;

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

    public void zeroMotors() {
        turretMotor.setPosition(0.0);
        pivotMotor.setPosition(0.0);
    }

    public void zeroMotorsMag() {
        turretMotor.setPosition(-20/360.0);
        pivotMotor.setPosition(0);
    }

    public void pivotDown(){
        pivotMotor.set(-0.05);
    }
    public void turretTurn(){
        turretMotor.set(0.09);
    }
    public void turretTurnSlow(){
        turretMotor.set(-0.04);
    }
    public void stopMotors() {
        turretMotor.set(0);
        pivotMotor.set(0);
    }

    public Command pivotDownCommand(){
        return this.runOnce(
            () -> pivotDown()
            );
    }

      public Command stopMotorsCommand(){
        return this.runOnce(
            () -> stopMotors()
            );
    }
    public Command topMotorsCommand(){
        return this.runOnce(
            () -> stopMotors()
            );
    }


    // Using WPILib Command Composition

    public Command autoTrackingHubCommand(char al, boolean t) {
        return this.run(
                () -> autoTrackingHub(al, t));
    }

    public void autoTrackingHub(char a, boolean tuck) {
        Pose2d robotPos = drivetrain.getState().Pose;

        // Get robot-relative speeds and transform them to field-relative
        var robotRelativeSpeeds = drivetrain.getState().Speeds;
        var fieldRelativeSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                robotPos.getRotation());

        double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
        double robotAngular = fieldRelativeSpeeds.omegaRadiansPerSecond;
        robotAngular = Math.toDegrees(robotAngular);
        double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

        // --- 2. PREDICTIVE TARGETING (Motion Compensation) ---
        // Initial distance estimate to get a baseline flight time

        Translation2d finalTarget = getFinalTarget(a, robotPos);

        distToHub = robotPos.getTranslation().getDistance(finalTarget);

        // double v = shooter.getSpeed();
        double v = SHOOTER_SPEED;
        double flightTime = distToHub / v; // Rough estimate of time-to-target

        // Calculate lead position: Target = Current - (Velocity * Time)
        Translation2d leadingTarget = new Translation2d(
                finalTarget.getX() - (robotVx * flightTime),
                finalTarget.getY() - (robotVy * flightTime));

        // --- 4. TURRET CONTROL (Azimuth) ---

        // Calculate angle to the LEADING target

        
                // X: 5.375 inches down of center
                // Y: 4.375 inches to left of center
        Translation2d turretOffset = new Translation2d(
            Units.inchesToMeters(-5.375),     // back
            Units.inchesToMeters(4.375)    // left ///////inverted?
        );

        Translation2d turretOffsetField = turretOffset.rotateBy(robotPos.getRotation());

        Translation2d turretFieldPosition = robotPos.getTranslation().minus(turretOffsetField);

            double angleToLeadRad = Math.atan2(
        leadingTarget.getY() - turretFieldPosition.getY(),
        leadingTarget.getX() - turretFieldPosition.getX());
      
        // double angleToLeadRad = Math.atan2(
        //         leadingTarget.getY() - robotPos.getY(),
        //         leadingTarget.getX() - robotPos.getX());


        //                 turretTargetDeg = MathUtil.inputModulus(
        // Units.radiansToDegrees(angleToLeadRad) - robotPos.getRotation().getDegrees() + TurretConstants.TURRET_OFFSET_DEG,
        // 0, 360
        // );

        // double turretError = MathUtil.inputModulus(turretTargetDeg - currentTurretAngle, -180, 180);

    //    turretTarget = MathUtil.inputModulus(
    //            Units.radiansToRotations(angleToLeadRad) - robotPos.getRotation().getRotations(),
    //            -1, 0);

        


        

       turretTarget = Units.radiansToRotations(angleToLeadRad) - robotPos.getRotation().getRotations() - (robotAngular * 0.00002);

        turretMotor.setControl(request.withPosition((turretTarget)));

        // --- 5. PIVOT CONTROL (Using Lookup Table) ---

        // 1. Get distance in meters (already calculated earlier)
        double distanceMeters = distToHub;

        SmartDashboard.putNumber("Dis", distanceMeters);

        if (tuck == true) {
             targetPitchDegrees = 0;
        } else {
            targetPitchDegrees = pivotLookup.getAngle(distanceMeters);
            targetPitchDegrees = MathUtil.clamp(targetPitchDegrees, 0, 45);
        }

        double pitchOnlyRotations = targetPitchDegrees / 360.0;

        double turretCurrentRotations = turretMotor.getPosition().getValueAsDouble();

        pivotMotor.setControl(request.withPosition(pitchOnlyRotations + ((turretCurrentRotations * 0.2088) - (robotAngular * 0.00003))));
    }

    private Translation2d getFinalTarget(char a, Pose2d robotPos) {
        Translation2d Blue_Left_Target = new Translation2d(2, 1);
        Translation2d Blue_Right_Target = new Translation2d(2, 7);
        Translation2d Red_Left_Target = new Translation2d(14.535, 7);
        Translation2d Red_Right_Target = new Translation2d(14.535, 1);

        Translation2d desiredHub;

        if (a == 'R')
            desiredHub = redHubMeters;
        else
            desiredHub = blueHubMeters;

        Translation2d finalTarget;

        if (alliance == 'R') {
            if (robotPos.getX() > 11.9) {
                finalTarget = desiredHub;
            } else if (robotPos.getY() > 4) {
                finalTarget = Red_Left_Target;
            } else {
                finalTarget = Red_Right_Target;
            }

        } else {
            if (robotPos.getX() < 4.7) {
                finalTarget = desiredHub;
            } else if (robotPos.getY() > 4) {
                finalTarget = Blue_Right_Target;
            } else {
                finalTarget = Blue_Left_Target;
            }
        }
        return finalTarget;
    }

    @Override
    public void periodic() {
        // --- Telemetry ---
        SmartDashboard.putNumber("Pivot/Current_Pitch_Deg", pivotMotor.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("Pivot/Target_Pitch_Deg", targetPitchDegrees);

        SmartDashboard.putNumber("Turret/Turrent_Target_Post_Config", turretMotor.getClosedLoopReference().getValueAsDouble());
    
        SmartDashboard.putNumber("Turret/Current_Turret", turretMotor.getPosition().getValueAsDouble()*360);
        SmartDashboard.putNumber("Turret/Target_Turret", turretTarget);
        SmartDashboard.putBoolean("Turret/Switch", limitSwitch.get());

    }
}
