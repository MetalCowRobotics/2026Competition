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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.Commands;
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
        shooter = new Shooter();
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

        // NamedCommands.registerCommand("Shoot", feeder.runFeederCommand().alongWith(shooter.startShooter()).alongWith(spindexer.runSpindexerCommand()).alongWith(intake.pivotAgitateCommand()));
        NamedCommands.registerCommand("Stop Shoot", shooter.shooterStop());
        NamedCommands.registerCommand("Intake", intake.startIntakeCommand());
        NamedCommands.registerCommand("Stop Intake", intake.endIntakeCommand());
        NamedCommands.registerCommand("Switch Down", turret.switchTurretCommand());

        NamedCommands.registerCommand("Shoot", shooter.startShooterCommand());
        NamedCommands.registerCommand("Home", intake.pivotStopAgitateCommand());

        // NamedCommands.registerCommand("Shooting", setStateCommand(RobotState.SHOOTING));
        // NamedCommands.registerCommand("Home", setStateCommand(RobotState.IDLE));
        // NamedCommands.registerCommand("Intaking", setStateCommand(RobotState.INTAKING));
       

        configureBindings();

        // intake.setDefaultCommand(stateMachineCommand());
        // shooter.setDefaultCommand(stateMachineCommand());
        // turret.setDefaultCommand(stateMachineCommand());
        // turret.setDefaultCommand(turret.autoTrackingHubCommand(alliance));
        // feeder.setDefaultCommand(stateMachineCommand());
        // spindexer.setDefaultCommand(stateMachineCommand());
    }

   

     private void setState(RobotState newState) {
        currentState = newState;
    }

    private Command setStateCommand(RobotState newState){
    return Commands.runOnce(() -> setState(newState));
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

        drivetrain.registerTelemetry(logger::telemeterize);

         joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );


        
       
        // OPERATOR COMMANDS
        // operatorController.x().onTrue(
        //     Commands.runOnce(() -> setState(RobotState.INTAKING))
        // );

        // operatorController.leftBumper().onTrue(
        //     Commands.runOnce(() -> setState(RobotState.SHOOTING))
        // );

        // operatorController.a().onTrue(
        //     Commands.runOnce(() -> setState(RobotState.REVERSING))
        // );

        // operatorController.rightBumper().onTrue(
        //     Commands.runOnce(() -> setState(RobotState.IDLE))
        // );

        //operatorController.y().onTrue(intake.pivotStart());

        //operatorController.a().onTrue(turret.zeroPivotCommand());

       // turret.setDefaultCommand(turret.autoTrackingHubCommand(alliance));

        operatorController.x().onTrue(intake.runIntakeCommand());
        operatorController.x().toggleOnFalse(intake.endIntakeCommand());

        operatorController.y().onTrue(shooter.startShooterCommand());
    
        operatorController.y().toggleOnFalse(shooter.shooterStop());


        //  if(shooter.getShooterCurrent()>20){
        //     feeder.runFeeder();
        //     spindexer.runSpindexer();
        // }else{
        //     feeder.stopFeeder();
        //     spindexer.stopSpindexer();
        // }

        operatorController.rightBumper().onTrue(turret.homePivotCommand());
         //operatorController.rightBumper().toggleOnFalse(turret.autoTrackingHubCommand(alliance));
        
        operatorController.leftBumper().onTrue(turret.passingCommand(pose, alliance).withTimeout(1).andThen(shooter.startShooterCommand()));

        // operatorController.y().onTrue(feeder.runFeederCommand().alongWith(shooter.startShooterCommand()).alongWith(spindexer.runSpindexerCommand()).alongWith(intake.pivotAgitateCommand()));
        // operatorController.y().toggleOnFalse(feeder.stopFeederCommand().alongWith(shooter.shooterStop()).alongWith(spindexer.stopSpindexerCommand()));

        operatorController.a().whileTrue(intake.pivotAgitateCommand());
        operatorController.a().toggleOnFalse(intake.pivotStopAgitateCommand());


        operatorController.rightTrigger().onTrue(turret.switchTurretCommand());

        operatorController.leftTrigger().onTrue(intake.reverseIntakeCommand());
        operatorController.leftTrigger().onFalse(intake.stopIntakeCommand());

        joystick.a().onTrue(intake.pivotUp());
        //operatorController.rightTrigger().onFalse(turret.switchTurretCommand());
        //operatorController.b().whileFalse(turret.autoTrackingHubCommand(alliance));
        
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

    private Command stateMachineCommand() {

    return Commands.run(() -> {

        SmartDashboard.putString("Robot State", currentState.name());

        switch (this.currentState) {

            case IDLE:

                intake.stopIntake();
                shooter.stopShooter();
                turret.zeroPivot();
                turret.zeroTurret();
                spindexer.stopSpindexer();
                feeder.stopFeeder();
                intake.pivotStopAgitate();

                break;


            case INTAKING:

                intake.startIntake();
                shooter.stopShooter();
                
                  if(pose.getY()>4 || pose.getY()<12.535){
                    turret.passing(pose, alliance);
                }else{
                    turret.autoTrackingHub(alliance);
                }
                    

                spindexer.stopSpindexer();
                feeder.stopFeeder();
                intake.pivotStopAgitate();


                break;

            case SHOOTING:
                shooter.startShooter();
                intake.stopIntake();

                // if(pose.getY()>4 || pose.getY()<12.535){
                //     turret.passing(pose, alliance);
                // }else{
                //     turret.autoTrackingHub(alliance);
                // }
                

                turret.autoTrackingHub(alliance);


                if(shooter.isAtSpeed()){
                    spindexer.runSpindexer();
                    feeder.runFeeder();
                }else{
                    spindexer.stopSpindexer();
                feeder.stopFeeder();
                }
                
                intake.pivotStopAgitate();

                break;

            case REVERSING:

                intake.reverseIntakeCommand();
                shooter.shooterStop();
                turret.zeroPivotCommand();
                turret.zeroTurretCommand();
                spindexer.reverseSpindexerCommand();
                feeder.reverseFeederCommand();
                intake.pivotStopAgitateCommand();

                break;
        }

    }, intake, shooter, turret, spindexer, feeder);
}
}
