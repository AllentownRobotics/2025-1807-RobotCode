// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCMDs.ClimbInCMD;
import frc.robot.commands.ClimbCMDs.ClimbOutCMD;
import frc.robot.commands.DrivetrainCMDs.DriveCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorIncrementDownCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorIncrementUpCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToHomeCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL1CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL2CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL3CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL4CMD;
import frc.robot.commands.PlacerCMDs.PlacerSetBothForwardCMD;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Placer;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;

public class RobotContainer {
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final TunerConstants tuner = new TunerConstants(); //test if this removes errors
    private final Elevator elevatorSubsystem = new Elevator();
    private final Placer placerSubsystem = new Placer();
    private final Climb climbSubsystem = new Climb();
    private final Hopper hopperSubsystem = new Hopper();
    private final Blinkin lightSubsystem = new Blinkin();
    //private final Vision limelightSubsystem = new Vision(drivetrain, placerSubsystem);

    //private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //private final SendableChooser<Command> autoChooser;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.maxDriveSpeed * 0.1).withRotationalDeadband(TunerConstants.maxDriveAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(TunerConstants.maxDriveSpeed);

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.driverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);

    public RobotContainer() {
        //autoChooser = AutoBuilder.buildAutoChooser("Name selected auto here");
        //SmartDashboard.putData("Auto chooser", autoChooser);
        //need to populate auto chooser still!!!

        configureBindings();

    }

    private void configureBindings() {
    /*
    * __________________________________ DRIVER CONTROLLER __________________________________
    */
    
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            /*drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * TunerConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * TunerConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )*/
        new DriveCMD(drivetrain, driverController)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        // DRIVETRAIN SYSID
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on right bumper press
        driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.enableSlowMode()));
        driverController.leftBumper().onFalse(drivetrain.runOnce(() -> drivetrain.disableSlowMode()));

        drivetrain.registerTelemetry(logger::telemeterize);

/*
* __________________________________ OPERATOR CONTROLLER __________________________________
*/

        // ELEVATOR SYSID
        operatorController.a().onTrue(Commands.runOnce(SignalLogger::start));
        operatorController.b().onTrue(Commands.runOnce(SignalLogger::stop));
        
        operatorController.back().and(operatorController.a()).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kForward));
        operatorController.back().and(operatorController.b()).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kReverse));
        operatorController.start().and(operatorController.a()).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
        operatorController.start().and(operatorController.b()).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));

        /*operatorController.y().whileTrue(new ElevatorToHomeCMD(elevatorSubsystem));
        operatorController.povDown().whileTrue(new ElevatorToL1CMD(elevatorSubsystem));
        operatorController.povRight().whileTrue(new ElevatorToL2CMD(elevatorSubsystem));
        operatorController.povLeft().whileTrue(new ElevatorToL3CMD(elevatorSubsystem));
        operatorController.povUp().whileTrue(new ElevatorToL4CMD(elevatorSubsystem));
        operatorController.x().whileTrue(new ElevatorIncrementDownCMD(elevatorSubsystem));
        operatorController.b().whileTrue(new ElevatorIncrementUpCMD(elevatorSubsystem));

        operatorController.leftBumper().whileTrue(new ClimbOutCMD(climbSubsystem));
        operatorController.rightBumper().whileTrue(new ClimbInCMD(climbSubsystem));

        operatorController.rightTrigger().whileTrue(new PlacerSetBothForwardCMD(placerSubsystem, operatorController.getRightTriggerAxis()));
        operatorController.rightStick().whileTrue(new PlacerSetBothForwardCMD(placerSubsystem, operatorController.getRightY())); // if joystick is pushed down (negative values) motors will run in reverse
    */
        }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
