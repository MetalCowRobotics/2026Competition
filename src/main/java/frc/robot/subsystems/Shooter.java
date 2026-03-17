package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    // private final Turret turret;
    private double targetPosition;
    private boolean shooterEnabled = false;

    public Shooter() {
        shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ONE_ID);
        shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TWO_ID);

        // Configure the motor
        configureMotors();
    }

   private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // 1. Ramp and PID
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0; // 1 second ramp
    var slot0 = config.Slot0;
    slot0.kP = 5; 
    slot0.kV = 130; 
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 45; // Limit motor heat TODO: Change 

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 45; // Limit battery draw TODO: Change
    // 2. Mechanics
    config.Feedback.SensorToMechanismRatio = 0.977778;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Shooters should coast!

    StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shooterMotor1.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure leader motor. Error: " + status.toString());
        }

        // Configure shooter motor two
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1; // 1 second ramp
    slot0.kP = 5; 
    slot0.kV = 0.12; 
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 15.0; // Limit motor heat TODO:Change back

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 15.0; // Limit battery draw TODO:Change back
    // 2. Mechanics
    config.Feedback.SensorToMechanismRatio = 0.977778;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Shooters should coast!
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shooterMotor2.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure follower motor. Error: " + status.toString());
        }
    // 5. Follower logic
    shooterMotor2.setControl(new StrictFollower(shooterMotor1.getDeviceID()));
}

    public void setTargetPosition(double positionMeters) {
        targetPosition = positionMeters;
    
    }

    @Override
    public void periodic() {
         // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter/Shooter_Velo_1", shooterMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Shooter_Velo_2", shooterMotor2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Shooter_Position_1", shooterMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Shooter_Position_2", shooterMotor2.getPosition().getValueAsDouble());
    }

    // @Override
    public void toggleShooter() {
        shooterEnabled = !shooterEnabled;
    }

    public void stop() {
        shooterMotor1.set(0);
    }

    public Command shooterStop()
    {
        return this.runOnce(
            () -> stop()
        );
    }

    // Create a reusable velocity request
    private final com.ctre.phoenix6.controls.VelocityVoltage velocityRequest = new com.ctre.phoenix6.controls.VelocityVoltage(20);

    public Command startShooter() {
        double targetRPS = (5000.0 / 60.0); // Converts 2000 RPM to RPS
        var velocityRequest =  new VelocityVoltage(30);
        
        return this.run(() -> {
            // Apply velocity control to motor 1 (motor 2 follows)
            shooterMotor1.setControl(velocityRequest);
        });
    }
    
}