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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final MotionMagicVoltage motionMagicRequest;
    private double targetPosition = 0;
    private static final double METERS_PER_ROTATION = 0.1595; // π * 0.0508m (circumference of 2-inch sprocket)

    public Elevator() {
        leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_ID);
        followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID);
        motionMagicRequest = new MotionMagicVoltage(0);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure gear ratio and mechanical conversion
        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 5.0; // 5:1 gear reduction

        // Configure Motion Magic
        MotionMagicConfigs mm = config.MotionMagic;
        mm.withMotionMagicCruiseVelocity(84.27)
          .withMotionMagicAcceleration(100.54)
          .withMotionMagicJerk(140.08);

        // Configure PID values
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.4;
        slot0.kV = 0.02;
        slot0.kA = 0.001;
        slot0.kP = 30;
        slot0.kI = 0;
        slot0.kD = .5;

        // Set to brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Configure leader motor
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = leaderMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure leader motor. Error: " + status.toString());
        }

        // Configure follower motor
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = followerMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure follower motor. Error: " + status.toString());
        }

        // Set follower to follow leader
        followerMotor.setControl(new StrictFollower(ElevatorConstants.LEADER_MOTOR_ID));
    }

    public void setTargetPosition(double positionMeters) {
        targetPosition = positionMeters;
        leaderMotor.setControl(motionMagicRequest.withPosition(positionMeters / METERS_PER_ROTATION).withSlot(0));
    }

    // Command wrappers for preset positions
    public Command goToL4Command() {
        return this.runOnce(() -> setTargetPosition(ElevatorConstants.L4_Distance));
    }

    public Command goToL3Command() {
        return this.runOnce(() -> setTargetPosition(ElevatorConstants.L3_Distance));
    }

    public Command goToL2Command() {
        return this.runOnce(() -> setTargetPosition(ElevatorConstants.L2_Distance));
    }

    public Command goToSourceCommand() {
        return this.runOnce(() -> setTargetPosition(ElevatorConstants.Source_Distance));
    }

    public Command goToRestCommand() {
        return this.runOnce(() -> setTargetPosition(ElevatorConstants.resetPos));
    }

    @Override
    public void periodic() {
        updateDashboard();
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("Elevator Position (m)", getPositionMeters());
        SmartDashboard.putNumber("Elevator Target (m)", targetPosition);
        SmartDashboard.putNumber("Elevator Velocity (m/s)", getVelocityMetersPerSecond());
    }

    public double getPositionMeters() {
        return leaderMotor.getRotorPosition().getValueAsDouble() * METERS_PER_ROTATION;
    }

    public double getVelocityMetersPerSecond() {
        return leaderMotor.getRotorVelocity().getValueAsDouble() * METERS_PER_ROTATION;
    }
} 