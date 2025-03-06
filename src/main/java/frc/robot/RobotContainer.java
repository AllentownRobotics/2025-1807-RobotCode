// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ClimbCMDs.ClimbInCMD;
import frc.robot.commands.ClimbCMDs.ClimbOutCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorIncrementCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToHomeCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL1CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL2CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL3CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL4CMD;
import frc.robot.commands.PlacerCMDs.CollectFromHopperCMD;
import frc.robot.commands.PlacerCMDs.EjectAlgaeCMD;
import frc.robot.commands.PlacerCMDs.PlaceCMD;
import frc.robot.commands.PlacerCMDs.ReverseFrontWheelsCMD;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Placer;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.0).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
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
    private final Vision visionSubsystem = new Vision();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

/*
* __________________________________ DRIVER CONTROLLER ________________________________
*/

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
        driverController.y().whileTrue(
            drivetrain.applyRequest(() ->
            driveRobotCentric.withRotationalRate(VisionConstants.rotationTargetingSpeed * visionSubsystem.getRightRotationPID())
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            )
        );

        driverController.x().whileTrue(
            drivetrain.applyRequest(() ->
            driveRobotCentric.withVelocityY(VisionConstants.translationTargetingSpeed * visionSubsystem.getRightXTranslationPID())
            .withVelocityX(0.0)
            .withRotationalRate(0.0)
            //.withRotationalRate(VisionConstants.rotationTargetingSpeed * visionSubsystem.getRightRotationPID())
            )
        );

        // Run Drivetrain SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

/*
* __________________________________ OPERATOR CONTROLLER __________________________________
*/

        // ELEVATOR SYSID
        /*operatorController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        operatorController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        operatorController.back().and(operatorController.a()).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kForward)).onFalse((new InstantCommand(() -> elevatorSubsystem.stopElevatorVolts(), elevatorSubsystem)));
        operatorController.back().and(operatorController.b()).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kReverse)).onFalse((new InstantCommand(() -> elevatorSubsystem.stopElevatorVolts(), elevatorSubsystem)));
        operatorController.start().and(operatorController.a()).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kForward)).onFalse((new InstantCommand(() -> elevatorSubsystem.stopElevatorVolts(), elevatorSubsystem)));
        operatorController.start().and(operatorController.b()).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kReverse)).onFalse((new InstantCommand(() -> elevatorSubsystem.stopElevatorVolts(), elevatorSubsystem)));
        */

        //testing
        //operatorController.b().onTrue(new ElevatorIncrementCMD(elevatorSubsystem,1));
        operatorController.x().onTrue(new ElevatorIncrementCMD(elevatorSubsystem, -1));

        operatorController.y().whileTrue(new ElevatorToHomeCMD(elevatorSubsystem));
        operatorController.povDown().whileTrue(new ElevatorToL1CMD(elevatorSubsystem));
        operatorController.povRight().whileTrue(new ElevatorToL2CMD(elevatorSubsystem));
        operatorController.povLeft().whileTrue(new ElevatorToL3CMD(elevatorSubsystem));
        operatorController.povUp().whileTrue(new ElevatorToL4CMD(elevatorSubsystem));

        operatorController.a().whileTrue(new CollectFromHopperCMD(placerSubsystem));
        operatorController.back().whileTrue(new PlaceCMD(placerSubsystem)); // change to read joystick values
        operatorController.y().whileTrue(new ReverseFrontWheelsCMD(placerSubsystem));
        operatorController.x().whileTrue(new EjectAlgaeCMD(placerSubsystem));
        
        operatorController.leftBumper().whileTrue(new ClimbOutCMD(climbSubsystem));
        operatorController.rightBumper().whileTrue(new ClimbInCMD(climbSubsystem)); 
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
