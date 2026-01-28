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

    /**
     * Sets pivot target angle in DEGREES
     * Automatically clamped between 0° and 30°
     */

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
}
