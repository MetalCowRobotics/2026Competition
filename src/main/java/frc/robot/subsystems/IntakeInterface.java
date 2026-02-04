package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;


interface IntakeInterface {

    public void configureMotors();

    public void setTargetAngleRadians(double radians);
    public Command intakeOut();
    public Command intakeIn();
    public Command agitateIntake();
    public Command startIntake();
    public Command stopIntake();

    public void periodic();
}
