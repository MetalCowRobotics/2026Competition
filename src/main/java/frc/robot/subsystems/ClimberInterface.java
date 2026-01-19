package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;


interface ClimberInterface {

    public void setTargetLocation();
    public Command goToRise();
    public Command goToRest();
    public Command ManualClimb();
    public double getCurrentAngle();

    public void setClimbSpeed(double speed);
    public void engagePneumatics(); 
    public void disengagePneumatics(); 
    public void reset(); 
    public void stopMotors(); 

    public boolean isPneumaticsEngaged(); 

    public void periodic();

}
