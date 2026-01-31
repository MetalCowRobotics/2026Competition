package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Turret extends SubsystemBase {

    private final TalonFX turretMotor;

    private double targetAngleDeg;
    private final PIDController pidController;

    public Turret() {

        turretMotor = new TalonFX(19);
        pidController = new PIDController(0.05, 0, 0);
        pidController.setTolerance(0.5);

        configureMotors();

        // Assume turret is physically at 0° on boot
        turretMotor.setPosition(0.0);
    }

    private void configureMotors() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 66.6667; // 66.6667:1 reduction 

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

         StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = turretMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println(
                "Turret motor config failed: " + status.toString()
            );
        }
    }


    public double getCurrentAngleDeg() {
        return turretMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean atTarget() {
        return pidController.atSetpoint();
    }
 

    public void setTargetPosition(double angleDeg) {

        pidController.setSetpoint(angleDeg);
    }

    public Command goToAngle(int angle) {
        return this.runOnce(
            // When the command starts, run the intake
            () -> setTargetPosition(angle)
        );
    }

    public Command goToAngle(Pose2d pose, char cha ) {
        return this.runOnce(
            // When the command starts, run the intake
            () -> setTargetPosition(getYawAngle(pose, cha))
        );
    }

    public double getYawAngle(Pose2d pose, char Alliance)
    {
        double angle = calculateTrajectory(pose, Alliance);
        angle = Math.toDegrees(angle);
        return angle;
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public Command stopCommand()
    {
       return this.runOnce( () -> turretMotor.set(0));
    }

    @Override
    public void periodic() {

        double currentAngle = getCurrentAngleDeg();

        double output = pidController.calculate(currentAngle);

        // Clamp PID output to motor-safe range
        output = MathUtil.clamp(output, -0.6, 0.6);

        turretMotor.set(output);

        /* ===== Dashboard ===== */
        SmartDashboard.putNumber("Turret Angle (deg)", currentAngle);
        SmartDashboard.putNumber("Turret Target (deg)", targetAngleDeg);
        SmartDashboard.putNumber("Turret PID Output", output);
        SmartDashboard.putBoolean("Turret At Target", atTarget());
    }


    public double calculateTrajectory(Pose2d robotPose, char alliance) {

        /* ================= Hub Poses ================= */

        // Blue Alliance Hub: (4.625, 4.035)
        Pose2d blueHub = new Pose2d(
                4.625,
                4.035,
                new Rotation2d()
        );

        // Red Alliance Hub: (11.92, 4.035)
        Pose2d redHub = new Pose2d(
                11.92,
                4.035,
                new Rotation2d()
        );

        Pose2d hub = (alliance == 'R') ? redHub : blueHub;

        /* ================= Distance to Target ================= */

        double dx = hub.getX() - robotPose.getX();
        double dy = hub.getY() - robotPose.getY();

        /* ================= Yaw ================= */

        double angle = Math.atan2(dy, dx);

        /* ================= Projectile Math ================= */

        return angle;
    }
}
