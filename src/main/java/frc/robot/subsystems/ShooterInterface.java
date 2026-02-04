package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;

interface ShooterInterface {

    public void configureMotors();
    public Command startShooter();
    public Command stopShooter();
    public void periodic();


    

}
