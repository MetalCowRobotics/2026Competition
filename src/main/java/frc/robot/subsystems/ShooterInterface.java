package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;

interface ShooterInterface {

    public double getCurrentAngle();
    public double getCurrentVelocity();
    public boolean isReady();

    public void setSpeed();
    public void setAngle();
    public void stop();

    public Command startShooterCommand();
    public Command stopShooterCommand();
    public Command runShooterAt(double speed);
    public Command setHoodAngle(double angle); 
    public Command shoot();

}
