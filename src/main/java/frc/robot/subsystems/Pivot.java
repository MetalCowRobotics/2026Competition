package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Pivot extends SubsystemBase implements PivotInterface{

    private final TalonFX pivotMotor;
    private final ShooterLookup shooterLookup;

    private double targetAngleDeg;
    private final PIDController pidController;
    private Transform2d robotRelativeTurretTransform;
    ShootingParams params;

    public Pivot() {

        pivotMotor = new TalonFX(16);
        pidController = new PIDController(0.05, 0, 0);
        pidController.setTolerance(0.5);
        shooterLookup = new ShooterLookup();
        params = new ShootingParams();
        robotRelativeTurretTransform = new Transform2d(1, 1, new Rotation2d());

        configureMotors();

        // Assume pivot is physically at 0° on boot
        pivotMotor.setPosition(0.0);
    }

    public void configureMotors() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 54.755; // 54.755:1 reduction 

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                ShooterConstants.PIVOT_MAX_ROT;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                ShooterConstants.PIVOT_MIN_ROT;


        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = pivotMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println(
                "Pivot motor config failed: " + status.toString()
            );
        }
    }


    public double getCurrentAngleDeg() {
        return pivotMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public boolean atTarget() {
        return pidController.atSetpoint();
    }

    public void setSpeed(double s)
    {
        pivotMotor.set(s);
    }


    public void setTargetPosition(double angleDeg) {
        targetAngleDeg = Math.max(
            ShooterConstants.PIVOT_MIN_DEG,
            Math.min(angleDeg, ShooterConstants.PIVOT_MAX_DEG)
        );

        pidController.setSetpoint(targetAngleDeg);
    }

    public Command goToAngle(int angle) {
        return this.runOnce(
            // When the command starts, run the intake
            () -> setTargetPosition(angle)
        );
    }

    public Command goToAngle(Pose2d pose, char alliance) {
        return this.runOnce(
            // When the command starts, run the intake
            () -> setTargetPosition(getPitchAngle(pose, alliance))
        );
    }

        public Command goToAngle(Pose2d pose, ChassisSpeeds speeds, char alliance) {
        return this.runOnce(
            // When the command starts, run the intake
            () -> setTargetPosition(getPitchAngle(pose, speeds, alliance)));
    }

    public double getPitchAngle(Pose2d pose, char alliance)
    {
        double angle = calculateTrajectory(pose, alliance);
        angle = Math.toDegrees(angle);
        targetAngleDeg = angle;
        return angle;
    }

    public double getPitchAngle(Pose2d pose, ChassisSpeeds speeds, char alliance)
    {
        params = calculateTrajectory(pose, speeds, alliance);
        return Math.toDegrees(params.pitchAngle);
    }

    public double getShooterSpeed(Pose2d pose, ChassisSpeeds speeds, char alliance)
    {
        params = calculateTrajectory(pose, speeds, alliance);
        return params.flywheelRpm;
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public Command stopCommand()
    {
       return this.runOnce( () -> pivotMotor.set(0));
    }

    @Override
    public void periodic() {

        double currentAngle = getCurrentAngleDeg();

        double output = pidController.calculate(currentAngle);

        // Clamp PID output to motor-safe range
        output = MathUtil.clamp(output, -0.6, 0.6);

        //pivotMotor.set(output); //TODO: Add this back when testing

        /* ===== Dashboard ===== */
        SmartDashboard.putNumber("Pivot Angle (deg)", currentAngle);
        SmartDashboard.putNumber("Pivot Target (deg)", targetAngleDeg);
        SmartDashboard.putNumber("Pivot PID Output", output);
        SmartDashboard.putBoolean("Pivot At Target", atTarget());
    }

public ShootingParams calculateTrajectory(
        Pose2d robotPose,
        ChassisSpeeds robotVelocity,
        char alliance) {

    ShootingParams tparams = new ShootingParams();

    /* ================= Constants ================= */

    final double g = 9.81;
    final double shooterHeight = 0.9;   // meters
    final double targetHeight = 2.64;   // meters
    final double fixedVelocity = 18.0;  // m/s (example fixed shooter velocity)

    /* ================= Hub Pose ================= */

    Pose2d blueHub = new Pose2d(4.625, 4.035, new Rotation2d());
    Pose2d redHub  = new Pose2d(11.92,  4.035, new Rotation2d());
    Pose2d hub = (alliance == 'R') ? redHub : blueHub;

    /* ================= Distance ================= */

    double dx = hub.getX() - robotPose.getX();
    double dy = hub.getY() - robotPose.getY();
    double distance = Math.hypot(dx, dy);

    double heightDifference = targetHeight - shooterHeight;

    /* ================= Solve For Hood Angle ================= */

    double v2 = fixedVelocity * fixedVelocity;

    double discriminant =
            v2 * v2
            - g * (g * distance * distance
                   + 2 * heightDifference * v2);

    if (discriminant < 0) {
        // Shot not possible at this velocity
        return tparams;
    }

    double sqrt = Math.sqrt(discriminant);

    // High arc solution (use +)
    double numerator = v2 + sqrt;
    double denominator = g * distance;

    double hoodAngle = Math.atan(numerator / denominator);

    tparams.pitchAngle = hoodAngle;
    return tparams;
} 


public double calculateTrajectory(Pose2d robotPose, char alliance) {

    /* ================= Constants ================= */

    double gravity = 10.0;          // m/s^2 (close enough to 9.8)
    double apexFraction = 0.7;      // fraction of distance where apex occurs
    double apexHeight = 2.25;       // meters

    /* ================= Hub Poses ================= */

    // Blue Alliance Hub: (4.625, 4.035)
    Pose2d blueHub = new Pose2d(
            4.625,
            4.035,
            new Rotation2d()
    );

    // Red Alliance Hub: (11.92, 4.035)
    Pose2d redHub = new Pose2d(
            11.92,
            4.035,
            new Rotation2d()
    );

    Pose2d hub = (alliance == 'R') ? redHub : blueHub;

    /* ================= Distance to Target ================= */

    double dx = hub.getX() - robotPose.getX();
    double dy = hub.getY() - robotPose.getY();

    double trajectoryDistance = Math.sqrt(dx * dx + dy * dy);

    /* ================= Projectile Math ================= */

    // t = sqrt(2h / g)
    double timeUntilApex = Math.sqrt((2.0 * apexHeight) / gravity);

    double distanceUntilApex = trajectoryDistance * apexFraction;

    double horizontalVelocity = distanceUntilApex / timeUntilApex;
    double verticalVelocity = gravity * timeUntilApex;

    /* ================= Pitch ================= */

    double pitchAngle = Math.atan2(verticalVelocity, horizontalVelocity);

    return pitchAngle;
}


// public ShootingParams calculateTrajectory(Pose2d robotPose,ChassisSpeeds robotVelocity, char alliance) {

//     /* ================= Hub Poses ================= */

//     Pose2d blueHub = new Pose2d(4.625, 4.035, new Rotation2d());
//     Pose2d redHub  = new Pose2d(11.92,  4.035, new Rotation2d());

//     Pose2d hub = (alliance == 'R') ? redHub : blueHub;

//     ShootingParams params = new ShootingParams();

//     /* ================= Turret Pose ================= */

//     Pose2d turretPose = robotPose.plus(robotRelativeTurretTransform);

//     /* ================= Distance to Hub ================= */

//     double dx = hub.getX() - turretPose.getX();
//     double dy = hub.getY() - turretPose.getY();
//     double totalDistance = Math.hypot(dx, dy);

//     /* ================= Robot Velocity ================= */

//     double robotSpeed = Math.hypot(robotVelocity.vxMetersPerSecond,
//                                    robotVelocity.vyMetersPerSecond);

//     double robotVelocityAngle =
//             Math.atan2(robotVelocity.vyMetersPerSecond,
//                        robotVelocity.vxMetersPerSecond);

//     double robotToTargetAngle = Math.atan2(dy, dx);

//     /* ================= Sideways Velocity ================= */

//     double vrs =
//             robotSpeed *
//             Math.sin(robotVelocityAngle - robotToTargetAngle);

//     /* ================= Yaw Compensation ================= */

//     double flightTime =
//             shooterLookup.calculateFlightTime(totalDistance);

//     params.yawAngle =
//             Math.atan2(vrs * flightTime, totalDistance);

//     /* ================= Pitch & RPM ================= */

//     params.pitchAngle =
//             shooterLookup.calculateHoodAngle(totalDistance);

//     params.flywheelRpm =
//             shooterLookup.calculateFlywheelVelocity(totalDistance);

//     /* ================= Telemetry ================= */

//     SmartDashboard.putNumber("Yaw Angle (rad)", params.yawAngle);
//     SmartDashboard.putNumber("Pitch Angle (rad)", params.pitchAngle);

//     return params;
//     }
}

// dev is awesome at coding you csn do thish great job!!!-mr, President