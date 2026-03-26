package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    private final TalonFX spindexerMotor;
    Shooter shooter;

    public Spindexer(Shooter shooter) {
        // Use a different CAN ID than your intake (e.g., 17)
        spindexerMotor = new TalonFX(31);
        this.shooter = shooter;
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Adjust ratio based on your gearbox (e.g., 10:1)
        config.Feedback.SensorToMechanismRatio = 1; 
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimit = 95;
        config.CurrentLimits.StatorCurrentLimit = 100;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = spindexerMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Could not configure Spindexer motor. Error: " + status.toString());
        }
    }

    public void runSpindexer() {
        spindexerMotor.set(SpindexerConstants.SPINDEXER_SPEED); // 50% power is usually enough for indexing
        
    }

    public void stopSpindexer(){
        spindexerMotor.set(SpindexerConstants.SPINDEXER_IDLE_SPEED);
    }

    public Command runSpindexerCommand() {
     
        return this.run(
            () -> runSpindexer() // 50% power is usually enough for indexing
        );
    }

    public Command reverseSpindexerCommand() {
        return this.runEnd(
            () -> spindexerMotor.set(SpindexerConstants.SPINDEXER_REVERSE_SPEED),
            () -> spindexerMotor.set(SpindexerConstants.SPINDEXER_IDLE_SPEED)
        );
    }

    public Command stopSpindexerCommand() {
        return this.runOnce(() -> stopSpindexer());
    }

    @Override
    public void periodic(){
        if(shooter.getSpeed()>ShooterConstants.SPEED_THRESHOLD_FOR_INTAKE){
            runSpindexer();
        }else{
            stopSpindexer();
        }
    }
}