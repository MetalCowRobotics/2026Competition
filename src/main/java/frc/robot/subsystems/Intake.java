package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase implements IntakeInterface {

    private final SparkMax intakeMotor;
    private final TalonFX pivotMotor;
    private final MotionMagicVoltage motionMagicRequest;
    double intakeSpeed = 0;
    double pivotTargetAngle = 0;


    public Intake() {
        intakeMotor = new SparkMax(
            IntakeConstants.INTAKE_MOTOR_ID,
            MotorType.kBrushless
        );

        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
        motionMagicRequest = new MotionMagicVoltage(0);

        configureMotors();
    }

    public void configureMotors() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        FeedbackConfigs feedback = pivotConfig.Feedback;
        feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEAR_RATIO;

        MotionMagicConfigs mm = pivotConfig.MotionMagic;
        mm.withMotionMagicCruiseVelocity(IntakeConstants.CRUISE_VELOCITY)
          .withMotionMagicAcceleration(IntakeConstants.MOTION_MAGIC_ACCELERATION)
          .withMotionMagicJerk(IntakeConstants.MOTION_MAGIC_JERK);

        Slot0Configs slot0 = pivotConfig.Slot0;
        slot0.kS = IntakeConstants.KS;
        slot0.kV = IntakeConstants.KV;
        slot0.kA = IntakeConstants.KA;
        slot0.kP = IntakeConstants.KP;
        slot0.kI = IntakeConstants.KI;
        slot0.kD = IntakeConstants.KD;
        slot0.kG = IntakeConstants.KG;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = pivotMotor.getConfigurator().apply(pivotConfig);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println(
                "Could not configure pivot motor: " + status.toString()
            );
        }

        intakeMotor.configure(
            intakeConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /* ---------------- Pivot Motor ---------------- */

    public void setTargetAngleRadians(double radians) {
        double mechanismRotations = radians / (2.0 * Math.PI);

        pivotMotor.setControl(
            motionMagicRequest
                .withPosition(mechanismRotations)
                .withSlot(0)
        );
    }

    public Command intakeOut() {
        this.pivotTargetAngle = IntakeConstants.INTAKE_OUT_RAD;
        return this.runOnce(() ->
            setTargetAngleRadians(IntakeConstants.INTAKE_OUT_RAD)
        );
    }

    public Command intakeIn() {
        this.pivotTargetAngle = IntakeConstants.INTAKE_IN_RAD;
        return this.runOnce(() ->
            setTargetAngleRadians(IntakeConstants.INTAKE_IN_RAD)
        );
    }

    public Command agitateIntake() {
        return Commands.repeatingSequence(
            intakeIn(),
            Commands.waitSeconds(IntakeConstants.TIME_BETWEEN_AGITATION),
            intakeOut(),
            Commands.waitSeconds(IntakeConstants.TIME_BETWEEN_AGITATION)
        );
    }

    /* ---------------- Intake Motor ---------------- */

    public Command startIntake() {
        this.intakeSpeed = IntakeConstants.INTAKE_SPEED;
        return this.runOnce(() ->
            intakeMotor.set(IntakeConstants.INTAKE_SPEED)
        );
    }

    public Command stopIntake() {
        this.intakeSpeed = 0;
        return this.runOnce(() ->
            intakeMotor.set(0)
        );
    }

    public double getPivotAngleRadians() {
    pivotMotor.getPosition().refresh();
    return pivotMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        SmartDashboard.putNumber("Target Intake Pivot Angle", pivotTargetAngle);
        SmartDashboard.putNumber("Current Intake Pivot Angle", getPivotAngleRadians());
    }
}
