package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;

public class AlignToTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> targetPoseSupplier;
    private Pose2d targetPose;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private static final double ACTIVATION_DISTANCE_METERS = 3;
    private static final double SLOW_DOWN_DISTANCE_METERS = 0.05048; // 12 inches in meters
    private static final double MAX_SPEED_FAR = 5.0; // Increased from 4.0 to 6.0 m/s
    private static final double MAX_SPEED_NEAR = 0.4; // Increased from 2.0 to 3.0 m/s
    private static final double MAX_ROTATION_SPEED_FAR = 8.0; // Increased from 4.0 to 8.0 rad/s
    private static final double MAX_ROTATION_SPEED_NEAR = 4.0; // Increased from 2.0 to 4.0 rad/s
    private boolean isRedAlliance;

    public AlignToTarget(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        
        xController = new PIDController(
            AlignmentConstants.XY_P,
            AlignmentConstants.XY_I,
            AlignmentConstants.XY_D
        );
        yController = new PIDController(
            AlignmentConstants.XY_P,
            AlignmentConstants.XY_I,
            AlignmentConstants.XY_D
        );
        rotationController = new PIDController(
            AlignmentConstants.ROTATION_P,
            AlignmentConstants.ROTATION_I,
            AlignmentConstants.ROTATION_D
        );

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        xController.setTolerance(AlignmentConstants.POSITION_TOLERANCE_METERS);
        yController.setTolerance(AlignmentConstants.POSITION_TOLERANCE_METERS);
        rotationController.setTolerance(Math.toRadians(AlignmentConstants.ROTATION_TOLERANCE_DEGREES));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetPose = targetPoseSupplier.get();
        isRedAlliance = DriverStation.getAlliance().isPresent() && 
                       DriverStation.getAlliance().get() == Alliance.Red;
        
        // If on red alliance, invert X and Y coordinates
      
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        rotationController.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        var currentPose = drivetrain.getState().Pose;
        
        // Calculate distance to target
        double distance = new Translation2d(
            currentPose.getX(), 
            currentPose.getY()
        ).getDistance(
            
            new Translation2d(
                
            targetPose.getX(), 
                targetPose.getY()
            )
        );

        // If we're too far away, end the command
        if (distance > ACTIVATION_DISTANCE_METERS) {
            return true;
        }

        // Otherwise, check if we've reached the target
        return xController.atSetpoint() &&
               yController.atSetpoint() &&
               rotationController.atSetpoint();
    }

    @Override
    public void execute() {
        var currentPose = drivetrain.getState().Pose;
        
        // Calculate distance to target
        double distance = new Translation2d(
            currentPose.getX(), 
            currentPose.getY()
        ).getDistance(
            new Translation2d(
                targetPose.getX(), 
                targetPose.getY()
            )
        );

        // Only run alignment if we're within activation distance
        if (distance <= ACTIVATION_DISTANCE_METERS) {
            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
            double rotationSpeed = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
            );

            // Invert X and Y speeds if on red alliance since we're using field-oriented control
            if (isRedAlliance) {
                xSpeed = -xSpeed;
                ySpeed = -ySpeed;
            }

            // Calculate speed limits based on distance
            double speedScale;
            double rotationScale;
            if (distance > SLOW_DOWN_DISTANCE_METERS) {
                // Far from target - use linear interpolation between max and min speeds
                speedScale = MathUtil.interpolate(MAX_SPEED_NEAR, MAX_SPEED_FAR, 
                    Math.min(distance / ACTIVATION_DISTANCE_METERS, 0.5));
                rotationScale = MathUtil.interpolate(MAX_ROTATION_SPEED_NEAR, MAX_ROTATION_SPEED_FAR,
                    Math.min(distance / ACTIVATION_DISTANCE_METERS, 0.5));
            } else {
                // Close to target - use precise control
                speedScale = MAX_SPEED_NEAR;
                rotationScale = MAX_ROTATION_SPEED_NEAR;
            }

            // Clamp speeds with adaptive limits
            xSpeed = MathUtil.clamp(xSpeed, -speedScale, speedScale);
            ySpeed = MathUtil.clamp(ySpeed, -speedScale, speedScale);
            rotationSpeed = MathUtil.clamp(rotationSpeed, -rotationScale, rotationScale);

            drivetrain.setControl(
                fieldCentric
                    .withVelocityX(xSpeed)
                    .withVelocityY(ySpeed)
                    .withRotationalRate(rotationSpeed)
            );
        } else {
            // Stop if we're too far
            drivetrain.setControl(
                fieldCentric
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(fieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
} 