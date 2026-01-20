package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements ShooterInterface {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;

    public static final int SHOOTER_LEFT_ID = 10;
    public static final int SHOOTER_RIGHT_ID = 11;

    private double speed = 0.0;
    private boolean shooterEnabled = false;

    public Shooter() {
        shooterMotor1 = new TalonFX(SHOOTER_LEFT_ID); 
        shooterMotor2 = new TalonFX(SHOOTER_RIGHT_ID);
    }

    @Override
    public void periodic() {
        if (shooterEnabled) {
            shooterMotor1.set(speed);
            shooterMotor2.set(speed);
        } else {
            shooterMotor1.set(0.0);
            shooterMotor2.set(0.0);
        }
        SmartDashboard.putNumber("Shooter Velocity (m/s)", getShooter1Velocity());
    }

    private double getShooter1Velocity() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    private double getShooter2Velocity() {
        return shooterMotor2.getVelocity().getValueAsDouble();
    }

    @Override
    public boolean getShooterSpunUp() {
        return shooterEnabled && speed == 1.0 && getShooter1Velocity() > 70.0 && getShooter2Velocity() > 70.0;
    }

    @Override
    public void setShootingSpeed() {
        speed = 1.0;
    }

    @Override
    public void setAmpSpeed() {
        speed = 0.40;
    }

    @Override
    public void toggleShooter() {
        shooterEnabled = !shooterEnabled;
    }

    @Override
    public void stopShooter() {
        shooterEnabled = false;
    }

    @Override
    public void startShooter() {
        shooterEnabled = true;
    }
}
