package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase implements ShooterInterface {
    private final SparkMax shooterMotor;
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private double targetPosition;
    private boolean shooterEnabled = false;
    private final MotionMagicVoltage motionMagicRequest;

    public Shooter() {
        shooterMotor = new SparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ONE_ID);
        shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TWO_ID);
        motionMagicRequest = new MotionMagicVoltage(0);

        // Configure the motor
        SparkMaxConfig config = new SparkMaxConfig();
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure gear ratio and mechanical conversion
        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 5.0; // 5:1 gear reduction

        MotionMagicConfigs mm = config.MotionMagic;
        mm.withMotionMagicCruiseVelocity(39.27)
          .withMotionMagicAcceleration(59.54)
          .withMotionMagicJerk(100.08);

       
        // Configure PID values
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.02;
        slot0.kA = 0.01;
        slot0.kP = 10;
        slot0.kI = 0;
        slot0.kD = 1.0;

        // Set to brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Configure shooter motor one
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shooterMotor1.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure leader motor. Error: " + status.toString());
        }

        // Configure shooter motor two
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shooterMotor2.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure follower motor. Error: " + status.toString());
        }

        shooterMotor2.setControl(new StrictFollower(14));
    }

    public void setTargetPosition(double positionMeters) {
        targetPosition = positionMeters;
        shooterMotor1.setControl(motionMagicRequest.withPosition(positionMeters / ShooterConstants.METERS_PER_ROTATION).withSlot(0));
    }

    @Override
    public void periodic() {
         // This method will be called once per scheduler run

    }

    @Override
    public void toggleShooter() {
        shooterEnabled = !shooterEnabled;
    }

    public Command stopIntakeCommand() {
        return this.runOnce( () -> shooterMotor1.set(0));
    }

    public Command startIntakeCommand(double speed) {
        return this.startEnd(
            // When the command starts, run the intake
            () -> shooterMotor1.set(speed),
            // When the command ends, stop the intake
            () -> shooterMotor1.set(0)
        );
    }

    public void stop() {
        shooterMotor1.set(0);
    }
}
