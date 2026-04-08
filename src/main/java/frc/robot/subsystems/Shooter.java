package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;


public class Shooter extends SubsystemBase {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final PivotLookup pivotLookup;
    private final char alliance;
    private final CommandSwerveDrivetrain drivetrain;
    public double speed;

    SlewRateLimiter limiter = new SlewRateLimiter(20);

    public Shooter(CommandSwerveDrivetrain drivetrain, char alliance) {
        shooterMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ONE_ID);
        shooterMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TWO_ID);
        pivotLookup = new PivotLookup();
        this.alliance = alliance;
        this.drivetrain  = drivetrain;

        // Configure the motor
        configureMotors();
    }

    public boolean isAtSpeed(){
        return shooterMotor1.getVelocity().getValueAsDouble() > 300 && shooterMotor1.getVelocity().getValueAsDouble() > 300;
    }

   private void configureMotors() {

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.Feedback.SensorToMechanismRatio = 0.97778;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
        shooterConfig.Slot0.kP = 24;
        shooterConfig.Slot0.kV = 20;
        shooterConfig.Slot0.kD = 0.1;
        shooterConfig.Slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        shooterConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1.0; // 1 second ramp
    
    //     var slot0 = config.Slot0;
    // slot0.kP = 7; 
    // slot0.kV = 130; 
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT; // Limit motor heat TODO: Change 

    // config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT; // Limit battery draw TODO: Change
    // // 2. Mechanics
    // config.Feedback.SensorToMechanismRatio = 0.977778;
    // config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Shooters should coast!

    StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = shooterMotor1.getConfigurator().apply(shooterConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure leader motor. Error: " + status.toString());
        }
        shooterMotor1.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig);
        shooterMotor2.setControl(new StrictFollower(shooterMotor1.getDeviceID()));
    }



    public double getShooterCurrent(){
        return shooterMotor1.getStatorCurrent().getValueAsDouble();
    }

    public double getSpeed(){
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    public void stopShooter() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public void startShooter(){

        Pose2d robotPos = drivetrain.getState().Pose;       

        Translation2d redHubMeters = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.845));
        Translation2d blueHubMeters = new Translation2d(Units.inchesToMeters(182.11),Units.inchesToMeters(158.845));
        
        double distToHub = 0;
        Translation2d desiredHub;

        if (alliance == 'R') desiredHub = redHubMeters;
        else desiredHub = blueHubMeters;

        distToHub = robotPos.getTranslation().getDistance(desiredHub);

        speed = pivotLookup.getPower(distToHub);

        double smoothSpeed1 = limiter.calculate(speed);
        
        shooterMotor1.set(smoothSpeed1);
        shooterMotor2.set(smoothSpeed1); 
    }

    public Command shooterStop(){
            return this.runOnce(
                () -> stopShooter()
            );
        }

         public Command shooterAutoStop(){
            return this.runOnce(
                () -> stopShooter()
            );
        }

       
    
        
    public Command startShooterCommand() {
            return this.startEnd(
            () -> startShooter(),
            () -> stopShooter()
            );
    }

    public Command shooterCommand() {
            return this.run(
            () -> startShooter());
    }

    public Command shooterAutoCommand() {
            return this.runOnce(
            () -> startShooter());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Current", getShooterCurrent());
        SmartDashboard.putNumber("Shooter Speed", speed);
        SmartDashboard.putNumber("Velocity", getSpeed());
    }
}