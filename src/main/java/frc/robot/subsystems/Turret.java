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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private final TalonFX turretMotor;

    public double targetAngleDeg = 0;
    public double currentAngleDeg = 0;

    // private final PIDController pidController;
    // private Transform2d robotRelativeTurretTransform;
    // private ShootingParams params;

    public Turret() {

        turretMotor = new TalonFX(30);
        
        // pidController = new PIDController(0.005, 0, 0.0002);
        // pidController.setTolerance(3);
        // pidController.enableContinuousInput(-180.0, 180.0);
        // robotRelativeTurretTransform = new Transform2d(0, 0.15, new Rotation2d());
        // params = new ShootingParams();

        configureMotors();

        // Assume turret is physically at 0° on boot
        //turretMotor.setPosition(0.0);
    }

    public void configureMotors() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        FeedbackConfigs feedback = config.Feedback;
        feedback.SensorToMechanismRatio = 8.555; //8.555:1 reduction 

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        //  StatusCode status = StatusCode.StatusCodeNotInitialized;
        // for (int i = 0; i < 5; i++) {
        //     status = turretMotor.getConfigurator().apply(config);
        //     if (status.isOK()) break;
        // }

        turretMotor.getConfigurator().apply(config);

        // if (!status.isOK()) {
        //     System.out.println(
        //         "Turret motor config failed: " + status.toString()
        //     );
        // }
    }

    public void setTargetPosition(double angleDeg){
            this.targetAngleDeg = angleDeg;
        }

    public double getCurrentAngleDeg() {
        return (turretMotor.getPosition().getValueAsDouble()/0.1182) * 360;
    }


    public void setSpeed(double targetSpeed){
        turretMotor.set(targetSpeed);
    }

    
    public Command setTargetAngle(double targetAngle){
        this.targetAngleDeg = targetAngle;
        return this.runOnce(() -> stopCommand());
    }
        
    public Command stopCommand(){
       return this.runOnce( () -> turretMotor.set(0));
    }

    @Override
    public void periodic() {

        double currentAngle = getCurrentAngleDeg();
        double targetAngle = this.targetAngleDeg;

        // double output = pidController.calculate(currentAngle, targetAngleDeg);

        turretMotor.set((targetAngle - currentAngle) * 0.0005);

        SmartDashboard.putNumber("Turret Angle (deg)", currentAngle);
        SmartDashboard.putNumber("Turret Target (deg)", targetAngle);
        // SmartDashboard.putBoolean("Turret At Target", atTarget());
    }

    
    // public boolean atTarget() {
    //     return pidController.atSetpoint();
    // }
    // public double normalizeAngleDeg(double angle)
    // {
    //     return MathUtil.inputModulus(angle, -180.0, 180.0);
    // }


 



    // public Command puttingPosition(double targetPosition){
    //      return this.run(
    //         // When the command starts, run the intake
    //         () -> setSpeed(PIDController.calculate(turretMotor.getPosition().getValueAsDouble(), targetPosition;
    //     );
    // }

    // // motor.set(pid.calculate(encoder.getDistance(), setpoint));

    //  public Command (double targetSpeed){
    //            return this.run(
    //         // When the command starts, run the intake
    //         () -> setSpeed(targetSpeed)
    //     );
    // }
 

  


    // public Command goToAngle(int angle) {
    //     return this.runOnce(
    //         // When the command starts, run the intake
    //         () -> setTargetPosition(angle)
    //     );
    // }

    // public Command goToAngle(Pose2d pose, char cha ) {
    //     return this.runOnce(
    //         // When the command starts, run the intake
    //         () -> setTargetPosition(getYawAngle(pose, cha))
    //     );
    // }

    // public Command goToAngle(Pose2d pose, ChassisSpeeds csp, char cha ) {
    //     return this.runOnce(
    //         // When the command starts, run the intake
    //         () -> setTargetPosition(getYawAngle(pose, csp, cha))
    //     );
    // }

    // public double getYawAngle(Pose2d pose, char alliance)
    // {
    //     double angleRad = calculateTrajectory(pose, alliance);
    //     double angleDeg = Math.toDegrees(angleRad);
    //     return normalizeAngleDeg(angleDeg);
    // }

    // public double getPitchAngle(Pose2d pose, ChassisSpeeds speeds, char alliance)
    // {
    //     params = calculateTrajectory(pose, speeds, alliance);
    //     return Math.toDegrees(params.pitchAngle);
    // }

    // public double getYawAngle(Pose2d pose, ChassisSpeeds cSpeeds , char all)
    // {
    //     params = calculateTrajectory(pose, cSpeeds, all);
    //     return Math.toDegrees(params.yawAngle);
    // }


    // public double getTargetAngleDeg() 
    // {
    //     double angle = turretMotor.getPosition().getValueAsDouble() * 360.0;
    //     return normalizeAngleDeg(angle);
    // }


   


    public double calculateTrajectory(Pose2d robotPose, char alliance) {

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

        /* ================= Yaw ================= */

        double angle = Math.atan2(dy, dx);

        /* ================= Projectile Math ================= */

        return angle;
    }
}

//     // public ShootingParams calculateTrajectory(
//     //     Pose2d robotPose,
//     //     ChassisSpeeds robotVelocity,
//     //     char alliance) {

//     // ShootingParams tparams = new ShootingParams();

//     /* ================= Constants ================= */

//     final double g = 9.81;
//     final double shooterHeight = 0.9;   // meters
//     final double targetHeight = 2.64;   // meters
//     final double fixedVelocity = 12.0;  // m/s (example fixed shooter velocity)

//     /* ================= Hub Pose ================= */

//     Pose2d blueHub = new Pose2d(4.625, 4.035, new Rotation2d());
//     Pose2d redHub  = new Pose2d(11.92,  4.035, new Rotation2d());
//     Pose2d hub = (alliance == 'R') ? redHub : blueHub;

//     /* ================= Distance ================= */

//     double dx = hub.getX() - robotPose.getX();
//     double dy = hub.getY() - robotPose.getY();
//     double distance = Math.hypot(dx, dy);

//     double heightDifference = targetHeight - shooterHeight;

//     /* ================= Solve For Hood Angle ================= */

//     double v2 = fixedVelocity * fixedVelocity;

//     double discriminant =
//             v2 * v2
//             - g * (g * distance * distance
//                    + 2 * heightDifference * v2);

//     if (discriminant < 0) {
//         // Shot not possible at this velocity
//         return tparams;
//     }

//     double sqrt = Math.sqrt(discriminant);

//     // High arc solution (use +)
//     double numerator = v2 + sqrt;
//     double denominator = g * distance;

//     double hoodAngle = Math.atan(numerator / denominator);

//     tparams.pitchAngle = hoodAngle;

//     /* ================= Flight Time ================= */

//     double flightTime =
//             distance / (fixedVelocity * Math.cos(hoodAngle));

//     /* ================= Robot Motion Compensation ================= */

//     double robotSpeed = Math.hypot(
//             robotVelocity.vxMetersPerSecond,
//             robotVelocity.vyMetersPerSecond);

//     double robotVelocityAngle = Math.atan2(
//             robotVelocity.vyMetersPerSecond,
//             robotVelocity.vxMetersPerSecond);

//     double robotToTargetAngle = Math.atan2(dy, dx);

//     double sidewaysVelocity =
//             robotSpeed *
//             Math.sin(robotVelocityAngle - robotToTargetAngle);

//     double yawAngle =
//             Math.atan2(sidewaysVelocity * flightTime, distance);

//     tparams.yawAngle = yawAngle;

//     SmartDashboard.putNumber("Pitch Angle", Math.toDegrees(tparams.pitchAngle));
//     SmartDashboard.putNumber("Yaw Angle", Math.toDegrees(tparams.yawAngle));

//     return tparams;
// }
// }
//     public double calculateFlightTime(
//         double velocity,       // m/s
//         double angleRadians,   // radians
//         double shooterHeight,  // meters
//         double targetHeight    // meters
// ) {
//     double g = 9.81;

//     double verticalVelocity = velocity * Math.sin(angleRadians);
//     double heightDifference = targetHeight - shooterHeight;

//     double discriminant =
//             verticalVelocity * verticalVelocity
//             - 2 * g * heightDifference;

//     if (discriminant < 0) {
//         // Target unreachable
//         return -1;
//     }

//     double sqrt = Math.sqrt(discriminant);

//     // Use larger root
//     double time =
//             (verticalVelocity + sqrt) / g;

//     return time;
// }

//     public ShootingParams calculateTrajectory(Pose2d robotPose,ChassisSpeeds robotVelocity, char alliance, double time) {

    // double gravity = 10.0;          // m/s^2 (close enough to 9.8)
    // double apexFraction = 0.7;      // fraction of distance where apex occurs
    // double apexHeight = 2.25;       // meters
    // double shooterHeight = 0;
    // double targetHeight = 0;
    // double shooterVelocity = 0;

    // /* ================= Hub Poses ================= */

    // // Blue Alliance Hub: (4.625, 4.035)
    // Pose2d blueHub = new Pose2d(
    //         4.625,
    //         4.035,
    //         new Rotation2d()
    // );

    // // Red Alliance Hub: (11.92, 4.035)
    // Pose2d redHub = new Pose2d(
    //         11.92,
    //         4.035,
    //         new Rotation2d()
    // );

    // Pose2d hub = (alliance == 'R') ? redHub : blueHub;

    // /* ================= Distance to Target ================= */

    // double dx = hub.getX() - robotPose.getX();
    // double dy = hub.getY() - robotPose.getY();

    // double trajectoryDistance = Math.sqrt(dx * dx + dy * dy);
    // double totalDistance = Math.hypot(dx, dy);

    // /* ================= Projectile Math ================= */

    // // t = sqrt(2h / g)
    // double timeUntilApex = Math.sqrt((2.0 * apexHeight) / gravity);

    // double distanceUntilApex = trajectoryDistance * apexFraction;

    // double horizontalVelocity = distanceUntilApex / timeUntilApex;
    // double verticalVelocity = gravity * timeUntilApex;

    // /* ================= Pitch ================= */

    // double pitchAngle = Math.atan2(verticalVelocity, horizontalVelocity);

    // /* ================= Robot Velocity ================= */

    // double robotSpeed = Math.hypot(robotVelocity.vxMetersPerSecond,
    //                                robotVelocity.vyMetersPerSecond);

    // double robotVelocityAngle =
    //         Math.atan2(robotVelocity.vyMetersPerSecond,
    //                    robotVelocity.vxMetersPerSecond);

    // double robotToTargetAngle = Math.atan2(dy, dx);

    // /* ================= Sideways Velocity ================= */

    // double vrs =
    //         robotSpeed *
    //         Math.sin(robotVelocityAngle - robotToTargetAngle);

    // /* ================= Yaw Compensation ================= */

    // double verticalV = shooterVelocity * Math.sin(Math.toRadians(pitchAngle));
    // double heightDifference = targetHeight - shooterHeight;

    // double discriminant =
    //         verticalV * verticalV
    //         - 2 * gravity * heightDifference;

    
    // double sqrt = Math.sqrt(discriminant);

    // // Use larger root
    // double flightTime =
    //         (verticalV + sqrt) / gravity;

    // params.yawAngle =
    //         Math.atan2(vrs * flightTime, totalDistance);

       
    // /* ================= Telemetry ================= */

    // SmartDashboard.putNumber("Yaw Angle (rad)", params.yawAngle);
    // SmartDashboard.putNumber("Pitch Angle (rad)", params.pitchAngle);

    // return params;
    // }
