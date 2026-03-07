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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1/2 of a rotation per second

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0); // Driver controller
    public final CommandXboxController operatorController = new CommandXboxController(1); // Operator controller

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Vision vision;
    private final Pivot pivot;
    private final Turret turret;
    private final Shooter shooter;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Feeder feeder;

    private Pose2d pose;
    private char alliance;
    private ChassisSpeeds cSpeeds;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<String> autoLocationChooser;

    public RobotContainer() {
        // Create vision subsystem after drivetrain
        vision = new Vision(drivetrain);
        pivot = new Pivot();
        turret = new Turret(drivetrain);
        shooter = new Shooter();
        intake = new Intake();
        spindexer = new Spindexer();
        feeder = new Feeder();
        
        pose = drivetrain.getState().Pose;
        cSpeeds = drivetrain.getState().Speeds;
        
        if(DriverStation.getAlliance().get() == Alliance.Red)
        {
            alliance = 'R';
        }
        else alliance = 'B';
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        autoLocationChooser = new SendableChooser<>();
        autoLocationChooser.addOption("Left", new String("Left"));
        autoLocationChooser.addOption("Center", new String("Center"));
        autoLocationChooser.addOption("Right", new String("Right"));

        SmartDashboard.putData("Auto Location", autoLocationChooser);

        configureBindings();
    }

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
        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );


        drivetrain.registerTelemetry(logger::telemeterize);


        // OPERATOR COMMANDS


        // 1. INTAKE TOGGLE (Button X)
        // Press once to start the intake, press again to stop.
        operatorController.x().toggleOnTrue(
            intake.runIntakeCommand()
        );

        // 2. SPINDEXER + FEEDER TOGGLE (Button Y)
        // Press once to start BOTH, press again to stop BOTH.
        operatorController.y().toggleOnTrue(
           spindexer.runSpindexerCommand()
               .alongWith(feeder.runFeederCommand())
       );


        // playing with controller feedback for set target positions (Turret)
        operatorController.a().whileTrue(
            Commands.runOnce(
                () -> turret.setTargetPosition(0),
                turret
            )
        );

        operatorController.y().whileTrue(
            Commands.runOnce(
                () -> turret.setTargetPosition(360),
                turret
            )
        );

        operatorController.b().toggleOnTrue(shooter.shoot());
    
    }

    public Command getAutonomousCommand() {
        try{
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
