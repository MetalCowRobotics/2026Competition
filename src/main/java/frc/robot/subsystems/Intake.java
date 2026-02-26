package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(16);
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // 5:1 gear reduction (Motor turns 5 times for every 1 mechanism rotation)
        config.Feedback.SensorToMechanismRatio = 0.0; 
        
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Retry logic to ensure config is applied
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Could not configure Intake motor. Error: " + status.toString());
        }
    }

    @Override
    public void periodic() {
        // Periodic code here (e.g., logging to SmartDashboard)
    }

    // --- Commands ---

   public Command runIntakeCommand() {
    // runEnd(StartAction, EndAction)
    return this.runEnd(
        () -> intakeMotor.set(-0.9), // What to do while button is held
        () -> intakeMotor.set(0)    // What to do when button is released
    );
}

    public Command stopIntakeCommand() {
        // Explicitly stops the motor
        return this.runOnce(() -> intakeMotor.set(0));
    }
}