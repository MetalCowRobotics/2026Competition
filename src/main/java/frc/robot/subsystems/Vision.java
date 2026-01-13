package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    
    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        updatePoseEstimates();
    }

    private void updatePoseEstimates() {
        boolean doRejectUpdate = false;

        // Front Left Limelight
        PoseEstimate fleft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fleft");
        if (fleft != null) {
            SmartDashboard.putString("Limelight Fleft", fleft.pose.toString());
            SmartDashboard.putNumber("Limelight Fleft.pose.x", fleft.pose.getX());
            SmartDashboard.putNumber("Limelight Fleft.pose.y", fleft.pose.getY());
            SmartDashboard.putNumber("Limelight Fleft Tag Count", fleft.tagCount);

            doRejectUpdate = shouldRejectUpdate(fleft, 4.0);

            if (!doRejectUpdate) {
                // Calculate standard deviations based on number of tags and distance
                double[] stdDevs = calculateStdDevs(fleft);
                
                drivetrain.addVisionMeasurement(
                    new Pose2d(
                        fleft.pose.getX(),
                        fleft.pose.getY(),
                        fleft.pose.getRotation()
                    ),
                    fleft.timestampSeconds,
                    VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2])
                );
            }
        }
        

        // Front Right Limelight
        doRejectUpdate = false;
        PoseEstimate fright = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fright");
        if (fright != null) {
            SmartDashboard.putString("Limelight Fright", fright.pose.toString());
            SmartDashboard.putNumber("Limelight Fright.pose.x", fright.pose.getX());
            SmartDashboard.putNumber("Limelight Fright.pose.y", fright.pose.getY());
            SmartDashboard.putNumber("Limelight Fright Tag Count", fright.tagCount);

            doRejectUpdate = shouldRejectUpdate(fright, 4);

            if (!doRejectUpdate) {
                double[] stdDevs = calculateStdDevs(fright);
                
                drivetrain.addVisionMeasurement(
                    new Pose2d(
                    fright.pose.getX(),
                    fright.pose.getY(),
                    fright.pose.getRotation()
                    ),
                    fright.timestampSeconds,
                    VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2])
                );
            }
        }
    }

    private double[] calculateStdDevs(PoseEstimate estimate) {
        double xyStdDev = 1.0;  // Start with high uncertainty
        double thetaStdDev = 1.0;

        if (estimate.tagCount >= 2) {
            // Multiple tags give us much better confidence
            xyStdDev = 0.3;
            thetaStdDev = 0.2;
            
            // Still adjust for distance to closest tag
            double minDist = Double.POSITIVE_INFINITY;
            for (var tag : estimate.rawFiducials) {
                minDist = Math.min(minDist, tag.distToCamera);
            }
            // Add 5% uncertainty per meter of distance
            double distanceMultiplier = 1.0 + (minDist * 0.05);
            xyStdDev *= distanceMultiplier;
            thetaStdDev *= distanceMultiplier;
            
        } else if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
            // Single tag - base confidence on distance and ambiguity
            double dist = estimate.rawFiducials[0].distToCamera;
            double ambiguity = estimate.rawFiducials[0].ambiguity;
            
            // Start with moderate uncertainty
            xyStdDev = 0.3;
            thetaStdDev = 0.2;
            
            // Add 10% uncertainty per meter of distance
            double distanceMultiplier = 1.0 + (dist * 0.1);
            // Add up to 100% more uncertainty based on ambiguity
            double ambiguityMultiplier = 1.0 + (ambiguity * 1.0);
            
            xyStdDev *= distanceMultiplier * ambiguityMultiplier;
            thetaStdDev *= distanceMultiplier * ambiguityMultiplier;
        }

        // Add telemetry for debugging
        SmartDashboard.putNumber("Vision XY StdDev", xyStdDev);
        SmartDashboard.putNumber("Vision Rotation StdDev", thetaStdDev);
        
        return new double[] {xyStdDev, xyStdDev, thetaStdDev};
    }

    private boolean shouldRejectUpdate(PoseEstimate estimate, double maxDistance) {
        if (estimate.tagCount == 0) {
            return true;
        }
        
        if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
            if (estimate.rawFiducials[0].ambiguity > 0.2) {
                return true;
            }
            if (estimate.rawFiducials[0].distToCamera > maxDistance) {
                return true;
            }
        }
        
        return false;
    }
} 