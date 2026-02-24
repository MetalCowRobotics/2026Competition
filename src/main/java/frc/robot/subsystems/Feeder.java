package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.SpindexerConstants;


public class Feeder extends SubsystemBase implements FeederInterface {
    private final SparkMax FeederMotor;

     public Feeder() {
        this.FeederMotor = new SparkMax(FeederConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
       
        configureMotors();
    }

    public void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(IdleMode.kCoast);
            //IntakeConstants.INTAKE_MOTOR_ID,
            //MotorType.kBrushless

        FeederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runFeeder(){
            return this.runOnce(
            () -> FeederMotor.set(FeederConstants.FEEDER_SPEED)
        );
    }
   public Command stopFeeder() {
        return this.runOnce(
           () -> FeederMotor.set(FeederConstants.FEEDER_IDLE_SPEED)
        );
    }
    public void periodic(){}
    
}
