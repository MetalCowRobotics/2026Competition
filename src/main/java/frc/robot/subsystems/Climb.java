package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import frc.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase {
    private final SparkMax climbMotor;
    private final SparkClosedLoopController closedLoopController;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private double targetLocation = 0;


    public Climb() {
        climbMotor = new SparkMax(ClimbConstants.CLIMB_CAN_ID, MotorType.kBrushless);
        closedLoopController = climbMotor.getClosedLoopController();

        absoluteEncoder = climbMotor.getAbsoluteEncoder();

        // Get the absolute encoder

        // Config
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kBrake)
              .smartCurrentLimit(50)
              .voltageCompensation(12);
        config.softLimit
              .reverseSoftLimit(0.260);
        config.softLimit
              .forwardSoftLimit(0.590);

        // Tell Spark to use absolute encoder
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(4)
            .i(0.0)
            .d(0.0)
            .outputRange(-1.0, 1.0)
            .maxMotion
            .maxVelocity(5000)
            .maxAcceleration(8000)
            .allowedClosedLoopError(0.01);

        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetLocation(double target) {
        this.targetLocation = target;
        closedLoopController.setReference(this.targetLocation, ControlType.kMAXMotionPositionControl);
    }

    // === Preset Commands ===
    public Command goToRise() {
        return this.runOnce(() -> setTargetLocation(ClimbConstants.climbTargetRise));
    }
    public Command goToRest() {
        return this.runOnce(() -> setTargetLocation(ClimbConstants.climbTargetRest));
    }
    public Command manualClimb(){
        return this.runOnce(() -> setTargetLocation(ClimbConstants.climbTargetClimb));
        // return this.startEnd(
        //     () -> climbMotor.set(-0.1), 
        //     () -> climbMotor.set(0.0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Absolute Encoder", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Climb Target Location", targetLocation);
    }

    public double getCurrentAngle() {
        return absoluteEncoder.getPosition();
    }
}
