package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private final TalonFX spindexerMotor;

    public Spindexer() {
        // Use a different CAN ID than your intake (e.g., 17)
        spindexerMotor = new TalonFX(31);
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Adjust ratio based on your gearbox (e.g., 10:1)
        config.Feedback.SensorToMechanismRatio = 1; 
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = spindexerMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Could not configure Spindexer motor. Error: " + status.toString());
        }
    }

    // --- Commands ---

    /**
     * Runs the spindexer forward. 
     * Using 'runEnd' ensures it stops immediately when the button is released.
     */
    public Command runSpindexerCommand() {
        return this.runEnd(
            () -> spindexerMotor.set(0.4), // 50% power is usually enough for indexing
            () -> spindexerMotor.set(0)
        );
    }

    /**
     * Runs the spindexer in reverse to clear jams.
     */
    public Command reverseSpindexerCommand() {
        return this.runEnd(
            () -> spindexerMotor.set(-0.3),
            () -> spindexerMotor.set(0)
        );
    }

    public Command stopSpindexerCommand() {
        return this.runOnce(() -> spindexerMotor.set(0));
    }
}