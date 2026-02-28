package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public final class ArmCommands {
    private final Intake intake;

    public ArmCommands(Intake intake) {
        this.intake = intake;
    }

    public Command stopIntake() {
        return intake.stopIntake();
    }

     public Command Shooter() {
        return intake.ShooterStop();
    }
}
