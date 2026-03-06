package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;

public class Feeder extends SubsystemBase {
    private final TalonFX feederMotor;

    public Feeder() {
        // Use a unique CAN ID (e.g., 18)
        feederMotor = new TalonFX(20);
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

    // --- Commands ---

    /**
     * Runs the feeder forward at full speed to launch into the shooter.
     */
    public Command runFeederCommand() {
        return this.runEnd(
            () -> feederMotor.set(FeederConstants.FEEDER_FAST_SPEED),
            () -> feederMotor.set(FeederConstants.FEEDER_IDLE_SPEED)
        );
    }

    /**
     * Runs the feeder slowly - useful for indexing or "unjamming".
     */
    public Command slowFeedCommand() {
        return this.runEnd(
            () -> feederMotor.set(FeederConstants.FEEDER_SLOW_SPEED),
            () -> feederMotor.set(FeederConstants.FEEDER_IDLE_SPEED)
        );
    }

    public Command stopFeederCommand() {
        return this.runOnce(() -> feederMotor.set(FeederConstants.FEEDER_IDLE_SPEED));
    }
}