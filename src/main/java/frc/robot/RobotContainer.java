// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCMDs.ClimbInCMD;
import frc.robot.commands.ClimbCMDs.ClimbOutCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorIncrementDownCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorIncrementUpCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToHomeCMD;
import frc.robot.commands.PlacerCMDs.PlacerSetBothForwardCMD;
import frc.robot.commands.PlacerCMDs.PlacerStopBothCMD;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Placer;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.driverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevatorSubsystem = new Elevator();
    private final Placer placerSubsystem = new Placer();
    private final Climb climbSubsystem = new Climb();
    private final Hopper hopperSubsystem = new Hopper();
    private final Blinkin blinkinSubsystem = new Blinkin();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        operatorController.povDown().onTrue(new ElevatorIncrementDownCMD(elevatorSubsystem));
        operatorController.povUp().onTrue(new ElevatorIncrementUpCMD(elevatorSubsystem));

        // ELEVATOR SYSID
        operatorController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        operatorController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        
        operatorController.back().and(operatorController.a()).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kForward)).onFalse((new InstantCommand(() -> elevatorSubsystem.stopElevatorVolts(), elevatorSubsystem)));
        operatorController.back().and(operatorController.b()).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kReverse)).onFalse((new InstantCommand(() -> elevatorSubsystem.stopElevatorVolts(), elevatorSubsystem)));
        operatorController.start().and(operatorController.a()).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kForward)).onFalse((new InstantCommand(() -> elevatorSubsystem.stopElevatorVolts(), elevatorSubsystem)));
        operatorController.start().and(operatorController.b()).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kReverse)).onFalse((new InstantCommand(() -> elevatorSubsystem.stopElevatorVolts(), elevatorSubsystem)));

        operatorController.povLeft().onTrue(new PlacerSetBothForwardCMD(placerSubsystem, .2)).onFalse(new PlacerStopBothCMD(placerSubsystem));
        operatorController.a().whileTrue(new PlacerSetBothForwardCMD(placerSubsystem, .2));

        if(false) {
        operatorController.y().whileTrue(new ElevatorToHomeCMD(elevatorSubsystem));
        
        operatorController.leftBumper().whileTrue(new ClimbOutCMD(climbSubsystem));
        operatorController.rightBumper().whileTrue(new ClimbInCMD(climbSubsystem)); 
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
