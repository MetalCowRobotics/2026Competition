package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SpindexerConstants;


public class Spindexer extends SubsystemBase implements SpindexerInterface {
    private final SparkMax spindexerMotor;

     public Spindexer() {
        this.spindexerMotor = new SparkMax(SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);
       
        configureMotors();
    }

    public void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(IdleMode.kCoast);

        spindexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runSpindexer(){
            return this.runOnce(
            () -> spindexerMotor.set(SpindexerConstants.SPINDEXER_SPEED)
        );
    }
   public Command stopSpindexer() {
        return this.runOnce(
           () -> spindexerMotor.set(SpindexerConstants.SPINDEXER_IDLE_SPEED)
        );
    }
    public void periodic(){}
    
}
