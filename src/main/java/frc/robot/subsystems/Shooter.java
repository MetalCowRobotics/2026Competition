package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase implements ShooterInterface {
    private final SparkMax shooterMotor;
    double shooterSpeed = 0;


    public Shooter() {
        shooterMotor = new SparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
      
        configureMotors();
    }

    public void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);

        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command stopShooter() {
        this.shooterSpeed = 0;
        return this.runOnce( () -> shooterMotor.set(0));
    }

    public Command startShooter() {
        this.shooterSpeed = ShooterConstants.SHOOTER_SPEED;
        return this.runOnce(
            // When the command starts, run the intake
            () -> shooterMotor.set(ShooterConstants.SHOOTER_SPEED)
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Shooter Speed", shooterSpeed);

    }
}
