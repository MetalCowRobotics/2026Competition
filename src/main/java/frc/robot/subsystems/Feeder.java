package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.ShooterConstants;

public class Feeder extends SubsystemBase {
    private final TalonFX feederMotor;
    Shooter shooter;

    public Feeder(Shooter shooter) {
        // Use a unique CAN ID (e.g., 18)
        feederMotor = new TalonFX(20);
        this.shooter = shooter;
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Feeders are often 1:1 or low reduction for speed
        config.Feedback.SensorToMechanismRatio = 1.0; 
        
        // Ensure this matches your physical build
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = feederMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Could not configure Feeder motor. Error: " + status.toString());
        }
    }
    
    public Command runFeederCommand() {
        return this.runEnd(
                        () -> feederMotor.set(FeederConstants.FEEDER_FAST_SPEED),
                        () -> feederMotor.set(FeederConstants.FEEDER_IDLE_SPEED)
                    );
        
        
    }

    public void runFeeder(){
        feederMotor.set(FeederConstants.FEEDER_FAST_SPEED);
    }

    public Command slowFeedCommand() {
        return this.runEnd(
            () -> feederMotor.set(FeederConstants.FEEDER_SLOW_SPEED),
            () -> feederMotor.set(FeederConstants.FEEDER_IDLE_SPEED)
        );
    }

    public Command stopFeederCommand() {
        return this.runOnce(() -> feederMotor.set(FeederConstants.FEEDER_IDLE_SPEED));
    }

    public void stopFeeder(){
       feederMotor.set(FeederConstants.FEEDER_IDLE_SPEED);
        
    }
    
    public Command reverseFeederCommand() {
        return this.startEnd(
            () -> feederMotor.set(-FeederConstants.FEEDER_FAST_SPEED),
            () -> feederMotor.set(0));
    }

    @Override
    public void periodic() {
        if(shooter.getSpeed() > ShooterConstants.SPEED_THRESHOLD_FOR_INTAKE){
            feederMotor.set(FeederConstants.FEEDER_FAST_SPEED);
        }else{
             feederMotor.set(FeederConstants.FEEDER_IDLE_SPEED);
        }

        SmartDashboard.putNumber("Feeder Speed", feederMotor.getVelocity().getValueAsDouble());
    } 
}