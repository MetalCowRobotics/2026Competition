// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.ColorSensorV3.Register;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1/2 of a rotation per second

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0); // Driver controller
    public final CommandXboxController operatorController = new CommandXboxController(1); // Operator controller

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision;
    private final Turret turret;
    private final Shooter shooter;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Feeder feeder;

    public Pose2d pose;
    public char alliance;
    public ChassisSpeeds cSpeeds;

    private enum RobotState {
        IDLE,
        INTAKING,
        SHOOTING,
        REVERSING,
        }
        
    public RobotState currentState;
    

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<String> autoLocationChooser;

    public RobotContainer() {
          if(DriverStation.getAlliance().get() == Alliance.Red)
        {
            alliance = 'R';
        }
        else alliance = 'B';

        // Create vision subsystem after drivetrain
        vision = new Vision(drivetrain);
        shooter = new Shooter(drivetrain,alliance);
        turret = new Turret(drivetrain,shooter,alliance);        
        intake = new Intake();
        spindexer = new Spindexer(shooter);
        feeder = new Feeder(shooter);

        currentState =  RobotState.IDLE;

        pose = drivetrain.getState().Pose;
        cSpeeds = drivetrain.getState().Speeds;
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        autoLocationChooser = new SendableChooser<>();
        autoLocationChooser.addOption("Left", new String("Left"));
        autoLocationChooser.addOption("Center", new String("Center"));
        autoLocationChooser.addOption("Right", new String("Right"));

        SmartDashboard.putData("Auto Location", autoLocationChooser);

        NamedCommands.registerCommand("Stop Shoot", shooter.shooterAutoStop().alongWith(intake.pivotStopAgitateCommand()));
        NamedCommands.registerCommand("Intake", intake.startIntakeCommand());
        NamedCommands.registerCommand("Stop Intake", intake.endIntakeCommand());
        NamedCommands.registerCommand("Shoot", shooter.shooterAutoCommand().withTimeout(2).alongWith(intake.pivotAgitateCommand()));
        NamedCommands.registerCommand("Home", intake.pivotStopAgitateCommand());
        NamedCommands.registerCommand("Start Turret", turret.autonomousTrackingHubCommand(alliance, false));
        NamedCommands.registerCommand("Stop Turret", turret.autonomousTrackingHubCommand(alliance, true));
       
        //NamedCommands.registerCommand("Trench", turret.zeroOnlyPivotCommand());

        configureBindings();
    }

    public Command getResetSequence() {
    return new SequentialCommandGroup(
        // 1. The Functional Command (Movement until Sensor)
        new FunctionalCommand(
            // On Start (init): Do nothing or prep
            () -> {}, 

            // Every Loop (execute): Run both motors
            () -> {
                turret.pivotDown();
                turret.turretTurn();
            },

            // On End: Stop the motors
            interrupted -> turret.stopMotors(),

            // Condition to Finish: Switch returns false
            () -> !turret.getSwitch(),

            // Requirements
            turret
        ),

        // 2. Wait for 0.2 seconds
        new WaitCommand(0.2),
        new FunctionalCommand(
            // On Start (init): Do nothing or prep
            () -> {}, 

            // Every Loop (execute): Run both motors
            () -> {
                turret.pivotDown();
                turret.turretTurnSlow();
            },

            // On End: Stop the motors
            interrupted -> turret.stopMotors(),

            // Condition to Finish: Switch returns false
            () -> !turret.getSwitch(),

            // Requirements
            turret
        ),
            new WaitCommand(0.2),
        // 3. Final Zeroing
        new InstantCommand(() -> turret.zeroMotors(), turret),
        Commands.run( () -> turret.autoTrackingHubCommand(alliance, false))
    );
}
//  public Command getPivotAndTurnCommand() {
//    return Commands.sequence(
//         // Race: Pivot and Turn until switch is hit
//         Commands.race(
//             Commands.run(() -> turret.pivotDown(), turret),
//             Commands.run(() -> turret.turretTurn(), turret),
//             Commands.waitUntil(() -> !turret.getSwitch()) // Fixed lowercase 'w'
//         ),

//         // Stop, Wait, and Zero
//         Commands.runOnce(() -> turret.stopMotors(), turret),
//         Commands.waitSeconds(0.2), 
//         Commands.runOnce(() -> turret.zeroMotors(), turret)
//     );
// }



    private void configureBindings() {
        
        
        // DRIVE COMMANDS
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);

         joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );


        
       
        // OPERATOR COMMAND

        operatorController.a().whileTrue(intake.pivotAgitateCommand());
        operatorController.a().whileFalse(intake.pivotStopAgitateCommand());

        operatorController.y().whileTrue(shooter.shooterCommand());
        operatorController.y().whileFalse(shooter.shooterStop());

        operatorController.x().toggleOnTrue(intake.runIntakeCommand());

        operatorController.leftTrigger().onTrue(intake.reverseIntakeCommand());
        operatorController.leftTrigger().onFalse(intake.stopIntakeCommand());

        operatorController.b().whileFalse(turret.autoTrackingHubCommand(alliance, false)); 
        operatorController.b().whileTrue(turret.autoTrackingHubCommand(alliance, true));
        operatorController.rightBumper().onTrue(getResetSequence());

    }

   

    public Command getAutonomousCommand() {
        try{

            if(autoChooser.getSelected().getName().endsWith("(No Mirror)")){
                return new PathPlannerAuto(autoChooser.getSelected().getName(), false);
            }

            if(autoLocationChooser.getSelected().equals("Right")){
                return new PathPlannerAuto(autoChooser.getSelected().getName(), false);
            }else{
                return new PathPlannerAuto(autoChooser.getSelected().getName(), true);
            }
        }catch(Exception e){
                DriverStation.reportError("PathPlanner ERROR: " + e.getMessage(), e.getStackTrace());
                return null;
        }

    }
}