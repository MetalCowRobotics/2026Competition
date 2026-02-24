package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;


interface FeederInterface {

    public void configureMotors();

    public Command runFeeder();
    public Command stopFeeder();

    public void periodic();
}
