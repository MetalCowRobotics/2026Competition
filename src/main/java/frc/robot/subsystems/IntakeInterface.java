package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;


interface IntakeInterface {

    public boolean hasGamePiece();

    public Command startIntakeCommand();
    public Command reverseIntakeCommand();
    public Command stopIntakeCommand();
    public void stop();
    
    public Command startIndexerCommand();
    public Command stopIndexerCommand();
    public Command reverseIndexerCommand();

    public void setIntakePower(double power);
    public void setIndexerPower(double power);

}
