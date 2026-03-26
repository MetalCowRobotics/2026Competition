package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX intakePivotMotor;

    public Intake() {
        intakeMotor = new TalonFX(16);
        intakePivotMotor = new TalonFX(21);
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration intakeconfig = new TalonFXConfiguration();

        // 5:1 gear reduction (Motor turns 5 times for every 1 mechanism rotation)
        intakeconfig.Feedback.SensorToMechanismRatio = 1; 
        
        intakeconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1.0;

        //intakePivotMotor.setPosition(2.68);
        intakePivotMotor.setPosition(IntakeConstants.PIVOT_INTAKE_UP);

        intakeconfig.Slot0.kP = IntakeConstants.KP; //TODO: Increase p

        // Retry logic to ensure intakeconfig is applied
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(intakeconfig);
            if (status.isOK()) break;
        }

        intakeconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        intakePivotMotor.getConfigurator().apply(intakeconfig);

        if (!status.isOK()) {
            System.out.println("Could not configure Intake motor. Error: " + status.toString());
        }
    }




    public void startIntake() {
        intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(IntakeConstants.INTAKE_IDLE_SPEED);
    }

    public void slowIntake() {
        intakeMotor.set(IntakeConstants.INTAKE_SLOW_SPEED);
    }

    public void reverseIntake() {
        intakeMotor.set(IntakeConstants.INTAKE_REVERSE_SPEED);
    }

    public Command stopIntakeCommand(){
        return this.runOnce(
            () -> stopIntake()
        );
    }


    public Command startIntakeCommand() {
        return this.runOnce(
            () -> startIntake()
        );
    }


   public Command runIntakeCommand() {
        return this.startEnd(
            () -> startIntake(),
            () -> stopIntake()
        );
    }

     public Command startSlowIntakeCommand() {
        return this.runOnce(
            () -> slowIntake()
        );
    }

    public Command pivotStart() {
        return this.runOnce(
            () -> intakePivotMotor.setControl(new PositionVoltage(IntakeConstants.PIVOT_INTAKE_UP))
        );
    }
    
    public void pivotStopAgitate(){
       intakePivotMotor.setControl(new PositionVoltage(IntakeConstants.PIVOT_INTAKE_UP));
    }

    public Command endIntakeCommand() {
        return this.runOnce(
            () -> stopIntake()
        );
    }
    
    public Command reverseIntakeCommand() {
        return this.startEnd(
            () -> reverseIntake(),
            () -> stopIntake()
        );
    }

    public Command pivotAgitateCommand() {
        return new ParallelCommandGroup(
            this.startEnd(
                () -> slowIntake(),
                () -> stopIntake()
            ),
            new RepeatCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> intakePivotMotor.setControl(
                        new PositionVoltage(IntakeConstants.PIVOT_INTAKE_DOWN)
                    )),
                    new WaitCommand(IntakeConstants.TIME_BTW_AGITATE),

                    new InstantCommand(() -> intakePivotMotor.setControl(
                        new PositionVoltage(IntakeConstants.PIVOT_INTAKE_UP)
                    )),
                    new WaitCommand(IntakeConstants.TIME_BTW_AGITATE)
                )
            )
        );
    }

    public Command pivotUp(){
        return this.runOnce(
            () -> intakePivotMotor.setControl(new PositionVoltage(IntakeConstants.PIVOT_INTAKE_UP))
        );
    }

    public Command pivotStopAgitateCommand(){
        return this.runOnce(
            () -> pivotStopAgitate()
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pivot Position", intakePivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Velocity", intakeMotor.getVelocity().getValueAsDouble());
    }
}