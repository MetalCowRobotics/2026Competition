package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;


interface SpindexerInterface {

    public void configureMotors();

    public Command runSpindexer();
    public Command stopSpindexer();

    public void periodic();
}
