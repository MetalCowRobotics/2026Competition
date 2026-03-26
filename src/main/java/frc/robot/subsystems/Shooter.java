package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SpindexerConstants;


public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;

    private boolean shooterEnabled = false;

    SlewRateLimiter limiter = new SlewRateLimiter(20);

    public Shooter() {
        shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ONE_ID);
        shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TWO_ID);

        // Configure the motor
        configureMotors();
    }

    public boolean isAtSpeed(){
        return shooterMotor1.getVelocity().getValueAsDouble() > 300 && shooterMotor1.getVelocity().getValueAsDouble() > 300;
    }

   private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // 1. Ramp and PID
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0; // 1 second ramp
    var slot0 = config.Slot0;
    slot0.kP = 5; 
    slot0.kV = 130; 
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT; // Limit motor heat TODO: Change 

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT; // Limit battery draw TODO: Change
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
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT; // Limit motor heat TODO:Change back

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT; // Limit battery draw TODO:Change back
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


    public double getShooterCurrent(){
        return shooterMotor1.getStatorCurrent().getValueAsDouble();
    }

    public double getSpeed(){
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    public void stopShooter() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public void startShooter(){
        double smoothSpeed1 = limiter.calculate(ShooterConstants.SHOOTER_SPEED);
        double smoothSpeed2 = limiter.calculate(ShooterConstants.SHOOTER_SPEED);
        
        shooterMotor1.set(smoothSpeed1);
        shooterMotor2.set(smoothSpeed2);
    }

    public Command shooterStop(){
            return this.runOnce(
                () -> stopShooter()
            );
        }
        
    public Command startShooterCommand() {
            return this.startEnd(
            () -> startShooter(),
            () -> stopShooter()
            );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Current", getShooterCurrent());
        SmartDashboard.putNumber("Velocity", getSpeed());
    }
}