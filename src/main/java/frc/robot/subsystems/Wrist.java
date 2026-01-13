package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final SparkMax wristMotor;
    private final SparkClosedLoopController closedLoopController;
    private double targetLocation = 0;
    private double desiredLocation = 0;
    private boolean isManualControl = false;

    public Wrist() {
        wristMotor = new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
        closedLoopController = wristMotor.getClosedLoopController();

        

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kBrake)
             .smartCurrentLimit(50)
             .voltageCompensation(12);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .d(0.08)
            .i(0.00006)
            .iZone(0.5)
            .outputRange(-0.5, 0.5)
            .maxMotion
            .maxVelocity(4200)
            .maxAcceleration(4000)
            .allowedClosedLoopError(.25);


        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        zeroEncoder();
    }

    public void setTargetLocation(double targetLocation) {
        if (!isManualControl) {
            this.desiredLocation = targetLocation;
        }
    }

    public void setManualControl(double speed) {
        if (isManualControl) {
            // Apply a deadband and limit the speed
            if (Math.abs(speed) < 0.1) {
                speed = 0;
            }
            // Limit the speed to 30% for safety
            speed = speed * 0.2;
            wristMotor.set(speed);
        }
    }

    public void toggleManualControl() {
        isManualControl = !isManualControl;
        if (!isManualControl) {
            // When switching back to preset mode, maintain current position
            desiredLocation = getCurrentAngle();
            targetLocation = desiredLocation;
        }
    }

    public boolean isInManualControl() {
        return isManualControl;
    }

    public void resume() {
        if (!isManualControl) {
            this.targetLocation = this.desiredLocation;
            closedLoopController.setReference(this.targetLocation, ControlType.kMAXMotionPositionControl);
        }
    }

    public void zeroEncoder() {
        wristMotor.getEncoder().setPosition(0);
        targetLocation = 0;
        desiredLocation = 0;
    }

    // Command wrappers for the preset positions
    public Command goToL4Command() {
        return this.runOnce(() -> setTargetLocation(WristConstants.L4_Angle));
    }

    public Command goToL3Command() {
        return this.runOnce(() -> setTargetLocation(WristConstants.L3_Angle));
    }

    public Command goToSourceCommand() {
        return this.runOnce(() -> setTargetLocation(WristConstants.Source_Angle));
    }

    public Command goToRestCommand() {
        return this.runOnce(() -> setTargetLocation(WristConstants.Rest_Angle));
    }

    public Command zeroEncoderCommand() {
        return this.runOnce(this::zeroEncoder);
    }

    public Command toggleManualControlCommand() {
        return this.runOnce(this::toggleManualControl);
    }

    public Command zeroEncoderAbsCommand() {
        return this.toggleManualControlCommand()
    .andThen(this.runOnce(this::toggleManualControl))
    .andThen(this.run(() -> wristMotor.set(-0.3))
        .until(() -> wristMotor.getAbsoluteEncoder().getPosition() > 0.953))
    .andThen(this.runOnce(() -> wristMotor.getEncoder().setPosition(0)))
    .andThen(this.runOnce(() -> wristMotor.set(0))); // stop motor

    }

    @Override
    public void periodic() {
        resume();
        SmartDashboard.putNumber("Wrist Encoder Reading", wristMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Abs Encoder Reading", wristMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Target Location", targetLocation);
        SmartDashboard.putBoolean("Wrist Manual Control", isManualControl);
    }

    public double getCurrentAngle() {
        return wristMotor.getEncoder().getPosition();
    }}