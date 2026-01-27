package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Pivot extends SubsystemBase implements PivotInterface {

    private final TalonFX pivotMotor;

    private double targetAngleDeg;
    private final MotionMagicVoltage motionMagicRequest;

    public Pivot() {

        pivotMotor = new TalonFX(16);
        motionMagicRequest = new MotionMagicVoltage(0);

        configureMotors();

        // Assume pivot is physically at 0° on boot
        pivotMotor.setPosition(0.0);
    }

    private void configureMotors() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 66.6667; // 66.6667:1 reduction

        MotionMagicConfigs mm = config.MotionMagic;
        mm.withMotionMagicCruiseVelocity(39.27)
          .withMotionMagicAcceleration(59.54)
          .withMotionMagicJerk(100.08);

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.02;
        slot0.kA = 0.01;
        slot0.kP = 1.0;
        slot0.kI = 0.0;
        slot0.kD = 0.0;

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
    public void setTargetPosition(double angleDeg) {

        double clampedDeg = Math.max(
            ShooterConstants.PIVOT_MIN_DEG,
            Math.min(angleDeg, ShooterConstants.PIVOT_MAX_DEG)
        );

        targetAngleDeg = clampedDeg;

        double targetRotations = clampedDeg / 360.0;

        pivotMotor.setControl(
            motionMagicRequest
                .withPosition(targetRotations)
                .withSlot(0)
        );
    }

    public Command goToAngle(int angle) {
        return this.startEnd(
            // When the command starts, run the intake
            () -> setTargetPosition(angle),
            // When the command ends, stop the intake
            () -> pivotMotor.set(0)
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
        // Optional: logging
        SmartDashboard.putNumber("Pivot Angle (deg)", 
        pivotMotor.getPosition().getValueAsDouble() * 360.0);
    }
}
