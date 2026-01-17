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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;


public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor;
    private final TalonFX leadMotor;
    private final TalonFX followMotor;
    private final MotionMagicVoltage motionMagicRequest;
    private double targetPosition = 0;
    private static final double METERS_PER_ROTATION = 0.1595; // π * 0.0508m (circumference of 2-inch sprocket)


    public Intake() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        leadMotor = new TalonFX(IntakeConstants.LEAD_MOTOR_ID);
        followMotor = new TalonFX(IntakeConstants.FOLLOW_MOTOR_ID);
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

        // Configure leader motor
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = leadMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure leader motor. Error: " + status.toString());
        }

        // Configure follower motor
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = followMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure follower motor. Error: " + status.toString());
        }

        // Set follower to follow leader
        followMotor.setControl(new StrictFollower(IntakeConstants.LEAD_MOTOR_ID));
    }

    public void setTargetPosition(double positionMeters) {
        targetPosition = positionMeters;
        leadMotor.setControl(motionMagicRequest.withPosition(positionMeters / METERS_PER_ROTATION).withSlot(0));
    }


    public Command intakeOut() {
        return this.runOnce(() -> setTargetPosition(IntakeConstants.INTAKE_OUT));
    }

    public Command intakeIn() {
        return this.runOnce(() -> setTargetPosition(IntakeConstants.INTAKE_IN));
    }

    public Command startIntakeCommand() {
        return this.startEnd(
            // When the command starts, run the intake
            () -> intakeMotor.set(IntakeConstants.INTAKE_SPEED),
            // When the command ends, stop the intake
            () -> intakeMotor.set(0)
        );
    }


    public Command stopIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(0)
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void stop() {
        intakeMotor.set(0);
    }
} 