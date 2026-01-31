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

public class Pivot extends SubsystemBase implements PivotInterface {

    private final TalonFX pivotMotor;

    private double targetAngleDeg;
    private final PIDController pidController;

    public Pivot() {

        pivotMotor = new TalonFX(16);
        pidController = new PIDController(0.05, 0, 0);
        pidController.setTolerance(0.5);

        configureMotors();

        // Assume pivot is physically at 0° on boot
        pivotMotor.setPosition(0.0);
    }

    private void configureMotors() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 66.6667; // 66.6667:1 reduction 

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                ShooterConstants.PIVOT_MAX_ROT;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                ShooterConstants.PIVOT_MIN_ROT;


        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = pivotMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println(
                "Pivot motor config failed: " + status.toString()
            );
        }
    }


    public double getCurrentAngleDeg() {
        return pivotMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean atTarget() {
        return pidController.atSetpoint();
    }
 

    public void setTargetPosition(double angleDeg) {

        targetAngleDeg = Math.max(
            ShooterConstants.PIVOT_MIN_DEG,
            Math.min(angleDeg, ShooterConstants.PIVOT_MAX_DEG)
        );

        pidController.setSetpoint(targetAngleDeg);
    }

    public Command goToAngle(int angle) {
        return this.runOnce(
            // When the command starts, run the intake
            () -> setTargetPosition(angle)
        );
    }

    public Command goToAngle() {
        return this.runOnce(
            // When the command starts, run the intake
            () -> setTargetPosition(getPitchAngle(null, 'R'))
        );
    }

    public double getPitchAngle(Pose2d pose, char Alliance)
    {
        ShootingParams params;
        params = calculateTrajectory(pose, Alliance);
        double angle = Math.toDegrees(params.pitchAngle);
        return angle;
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public Command stopCommand()
    {
       return this.runOnce( () -> pivotMotor.set(0));
    }

    @Override
    public void periodic() {

        double currentAngle = getCurrentAngleDeg();

        double output = pidController.calculate(currentAngle);

        // Clamp PID output to motor-safe range
        output = MathUtil.clamp(output, -0.6, 0.6);

        pivotMotor.set(output);

        /* ===== Dashboard ===== */
        SmartDashboard.putNumber("Pivot Angle (deg)", currentAngle);
        SmartDashboard.putNumber("Pivot Target (deg)", targetAngleDeg);
        SmartDashboard.putNumber("Pivot PID Output", output);
        SmartDashboard.putBoolean("Pivot At Target", atTarget());
    }

    public static class ShootingParams {
        public double yawAngle;    // radians
        public double pitchAngle;  // radians
    }

    public ShootingParams calculateTrajectory(Pose2d robotPose, char alliance) {

        /* ================= Constants ================= */

        double gravity = 10.0;          // m/s^2 (close enough to 9.8)
        double apexFraction = 0.7;      // fraction of distance where apex occurs
        double apexHeight = 2.25;       // meters

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

        ShootingParams params = new ShootingParams();

        /* ================= Distance to Target ================= */

        double dx = hub.getX() - robotPose.getX();
        double dy = hub.getY() - robotPose.getY();

        double trajectoryDistance = Math.sqrt(dx * dx + dy * dy);

        /* ================= Yaw ================= */

        params.yawAngle = Math.atan2(dy, dx);

        /* ================= Projectile Math ================= */

        // t = sqrt(2h / g)
        double timeUntilApex = Math.sqrt((2.0 * apexHeight) / gravity);

        double distanceUntilApex = trajectoryDistance * apexFraction;

        double horizontalVelocity = distanceUntilApex / timeUntilApex;
        double verticalVelocity = gravity * timeUntilApex;

        /* ================= Pitch ================= */

        params.pitchAngle = Math.atan2(verticalVelocity, horizontalVelocity);

        /* ================= Telemetry ================= */

        // Replace with SmartDashboard / AdvantageKit / custom logger
        // tkit.RecordOutput("Yaw Angle", params.yawAngle);
        // tkit.RecordOutput("Pitch Angle", params.pitchAngle);
        // tkit.RecordOutput("Apex Fraction", apexFraction);
        // tkit.RecordOutput("Apex Height", apexHeight);

        return params;
    }
}
