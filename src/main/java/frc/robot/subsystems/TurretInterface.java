package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

interface TurretInterface {

    public void configureMotors();

    public double normalizeAngleDeg(double angle);
    public double getCurrentAngleDeg();
    public boolean atTarget();
    public void setTargetPosition(double angleDeg);
    public Command goToAngle(int angle);
    public Command goToAngle(Pose2d pose, char cha );
    public double getYawAngle(Pose2d pose, char alliance);
    public double getTargetAngleDeg();
    public Command stopCommand();
    public double calculateTrajectory(Pose2d robotPose, char alliance);

    public void periodic();
    
}
