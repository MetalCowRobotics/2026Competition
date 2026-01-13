package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.AlignmentConstants;

public class LEDDefaultCommand extends Command {
    private final LEDSubsystem ledSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final double proximityThreshold = 3.0; // 1 meter threshold
    private final double rotationThreshold = 5.0; // 5 degrees threshold
    private final double translationThreshold = 0.1; // 10cm threshold for final alignment

    public LEDDefaultCommand(
            LEDSubsystem ledSubsystem, 
            CommandSwerveDrivetrain drivetrain,
            CommandXboxController driverController,
            CommandXboxController operatorController) {
        this.ledSubsystem = ledSubsystem;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.operatorController = operatorController;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d nearestTarget = AlignmentConstants.findClosestTarget(currentPose);
        
        if (nearestTarget != null) {
            double distance = currentPose.getTranslation().getDistance(nearestTarget.getTranslation());
            double angleError = Math.abs(nearestTarget.getRotation().minus(currentPose.getRotation()).getDegrees());
                
                // If we're close enough AND properly aligned
                if (distance < translationThreshold && angleError < rotationThreshold) {
                    ledSubsystem.setGreen();  // Red when fully aligned
                    // Activate rumble on both controllers
                    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                    operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                } else if (distance < proximityThreshold) {
                    ledSubsystem.setWhite();  // Green when close but not fully aligned
                    // Stop rumble
                    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                } else {
                ledSubsystem.setRed();  // White when not in range
                // Stop rumble
                driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            }}
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure to stop rumble when command ends
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
} 

//TNT