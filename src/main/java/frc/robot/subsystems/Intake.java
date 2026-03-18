package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        //intakePivotMotor.setPosition(2.68); //TODO: RESET THIS
        intakePivotMotor.setPosition(0);

        intakeconfig.Slot0.kP=20;

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pivot Value", intakePivotMotor.getPosition().getValueAsDouble());
    }


   public Command runIntakeCommand() {
        return this.startEnd(
            () -> intakeMotor.set(IntakeConstants.INTAKE_SPEED),
            () -> intakeMotor.set(0)
        );
    }

       public Command startIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(IntakeConstants.INTAKE_SPEED)
        );
    }

     public Command startSlowIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(IntakeConstants.INTAKE_SLOW_SPEED)
        );
    }

    public void pivotAgitate(){

         //intakePivotMotor.setControl(new PositionVoltage(1));
        if(intakePivotMotor.getPosition().getValueAsDouble() < 0.1) intakePivotMotor.setControl(new PositionVoltage(1));
        if(intakePivotMotor.getPosition().getValueAsDouble() > 0.9) intakePivotMotor.setControl(new PositionVoltage(0));
        intakeMotor.set(-0.3);
    }

    public Command pivotStart() {
        return this.runOnce(
            () -> intakePivotMotor.setControl(new PositionVoltage(0))
        );
    }
    
    public void pivotStopAgitate(){
       intakePivotMotor.setControl(new PositionVoltage(0));
    }

    public Command endIntakeCommand() {
        return this.runOnce(
            () -> intakeMotor.set(0)
        );
    }
    
    // use for whileTrue
    public Command reverseIntakeCommand() {
        return this.startEnd(
            () -> intakeMotor.set(IntakeConstants.INTAKE_REVERSE_SPEED),
            () -> intakeMotor.set(0)
        );
    }

    public Command pivotAgitateCommand()
    {
        return this.run(
            () -> pivotAgitate()
        );
    }

    public Command pivotStopAgitateCommand()
    {
        return this.run(
            () -> pivotStopAgitate()
        );
    }
}