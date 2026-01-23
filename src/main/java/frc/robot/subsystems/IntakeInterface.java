package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;


interface IntakeInterface {

    public void configureMotors();

    public void setTargetPosition(double positionMeters);

    public Command intakeOut();
    public Command intakeIn();
    public Command agitateIntake();
    public Command startIntakeCommand();
    public Command stopIntakeCommand();

    public void periodic();
    public void stop();
}
