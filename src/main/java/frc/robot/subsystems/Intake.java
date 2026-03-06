package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(16);
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // 5:1 gear reduction (Motor turns 5 times for every 1 mechanism rotation)
        config.Feedback.SensorToMechanismRatio = 1; 
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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


   public Command runIntakeCommand() {
        return this.startEnd(
            () -> intakeMotor.set(IntakeConstants.INTAKE_SPEED),
            () -> intakeMotor.set(0)
        );
    }

    public Command reverseIntakeCommand() {
        return this.startEnd(
            () -> intakeMotor.set(IntakeConstants.INTAKE_REVERSE_SPEED),
            () -> intakeMotor.set(0)
        );
    }
}