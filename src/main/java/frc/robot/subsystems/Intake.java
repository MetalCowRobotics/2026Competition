package frc.robot.subsystems;

import frc.robot.Robot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor;
    private static final int INTAKE_MOTOR_ID = 17; // Adjust this ID as needed
    private static final double INTAKE_SPEED = 0.9; // 70% speed for intake
    private static final double REVERSE_SPEED = -0.9; // 70% speed for outtake

    public Intake() {
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushed);
        
        // Configure the motor
        SparkMaxConfig config = new SparkMaxConfig();
    }

    public Command startIntakeCommand() {
        return this.startEnd(
            // When the command starts, run the intake
            () -> intakeMotor.set(INTAKE_SPEED),
            // When the command ends, stop the intake
            () -> intakeMotor.set(0)
        );
    }

    public Command IntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(INTAKE_SPEED)
        );
    }

    public Command reverseIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(REVERSE_SPEED)
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