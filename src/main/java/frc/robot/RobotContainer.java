// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PlacerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.BlinkinConstants.LEDPattern;
import frc.robot.commands.ClimbCMDs.ClimbInCMD;
import frc.robot.commands.ClimbCMDs.ClimbOutCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorIncrementCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToHomeCMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL1CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL2CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL3CMD;
import frc.robot.commands.ElevatorCMDs.ElevatorToL4CMD;
import frc.robot.commands.PlacerCMDs.CollectFromHopperCMD;
import frc.robot.commands.PlacerCMDs.PlaceCMD;
import frc.robot.commands.TargetingCMDs.TargetCMD;
import frc.robot.commands.TargetingCMDs.TargetWithLEDs;
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
    public double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double slowDriveSpeed = MaxSpeed * TunerConstants.slowDriveScalingConstant;
    private double slowAngularRate = MaxAngularRate * TunerConstants.slowDriveScalingConstant;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric slowDriveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(slowDriveSpeed * 0.1)
            .withRotationalDeadband(slowAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.0).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.driverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);

    private final SendableChooser<Command> autoChooser;


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Elevator elevatorSubsystem = new Elevator();
    private final Placer placerSubsystem = new Placer();
    private final Climb climbSubsystem = new Climb();
    private final Hopper hopperSubsystem = new Hopper();
    private final Blinkin blinkinSubsystem = new Blinkin();
    private final Vision visionSubsystem = new Vision();

    public RobotContainer() {

        
        /*  You must register your commands with NamedCommands (i.e. with pathplanner) so that they can be used
            by Event Markers in a path or as a command in an Auto.  Commands not registered will be ignored (TRAIF -- need to verify). */

        NamedCommands.registerCommand("ElevatorToHome", new ElevatorToHomeCMD(elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToL1", new ElevatorToL1CMD(elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToL2", new ElevatorToL2CMD(elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToL3", new ElevatorToL3CMD(elevatorSubsystem));
        NamedCommands.registerCommand("ElevatorToL4", new ElevatorToL4CMD(elevatorSubsystem));
        NamedCommands.registerCommand("PlaceToL1", new PlaceCMD(placerSubsystem, PlacerConstants.placerFrontMotorSpeed, PlacerConstants.placerBackMotorSpeed).withTimeout(.5)); // TRAIF figure out and put in Constants.java
        NamedCommands.registerCommand("PlaceToL2", new PlaceCMD(placerSubsystem, PlacerConstants.placerFrontMotorSpeed, PlacerConstants.placerBackMotorSpeed).withTimeout(.5)); //TRAIF figure out and put in Constants.java
        NamedCommands.registerCommand("PlaceToL3", new PlaceCMD(placerSubsystem, PlacerConstants.placerFrontMotorSpeed, PlacerConstants.placerBackMotorSpeed).withTimeout(.5)); //TRAIF figure out and put in Constants.java
        NamedCommands.registerCommand("PlaceToL4", new PlaceCMD(placerSubsystem, PlacerConstants.placerFrontMotorSpeed, PlacerConstants.placerBackMotorSpeed).withTimeout(.5)); //TRAIF figure out and put in Constants.java
        NamedCommands.registerCommand("CollectFromHopper", new CollectFromHopperCMD(placerSubsystem));
        NamedCommands.registerCommand("ElevatorWaitforL4", new WaitUntilCommand(elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L4Position)));
        NamedCommands.registerCommand("ElevatorWaitforL3", new WaitUntilCommand(elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L3Position)));
        NamedCommands.registerCommand("ElevatorWaitforL2", new WaitUntilCommand(elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L2Position)));
        NamedCommands.registerCommand("ElevatorWaitforL1", new WaitUntilCommand(elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.L1Position)));
        NamedCommands.registerCommand("ElevatorWaitforHome", new WaitUntilCommand(elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.homePosition)));

 
        NamedCommands.registerCommand("HopperWaitForCoralCollected", new WaitUntilCommand(() -> hopperSubsystem.isCoralCollected())); // TRAIF -- does this work?

        NamedCommands.registerCommand("LEDPatternOff", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.OFF), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternIdle", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.IDLE), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternAlertHumanPlayer", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.ALERT_HUMAN_PLAYER), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternAlignedWithReef", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.ALIGNED_WITH_REEF), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternCelebrate", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.CELEBRATE), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternCoralIndexed", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.CORAL_INDEXED), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternElevatorAtPosition", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.ELEVATOR_AT_DESIRED_POSITION), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternClimbing", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.CLIMB_COMPLETE), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternClimbCompleteRed", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.CLIMB_COMPLETE_RED), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternClimbCompleteBlue", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.CLIMB_COMPLETE_BLUE), blinkinSubsystem));
        NamedCommands.registerCommand("LEDPatternClimbComplete", new InstantCommand(() -> blinkinSubsystem.setPattern(Constants.BlinkinConstants.LEDPattern.CLIMB_COMPLETE), blinkinSubsystem));

        //NamedCommands.registerCommand("testinghopper", new HopperTest(hopperSubsystem).andThen(new CollectFromHopperCMD(placerSubsystem)));

        NamedCommands.registerCommand("TargetReefLeft", new TargetCMD(
            visionSubsystem, drivetrain,
            driverController,
            VisionConstants.targetingLeftTranslationOffset).until(() -> visionSubsystem.isNearPoseFrontTargetSpace(new Pose2d(new Translation2d(
                VisionConstants.targetingLeftTranslationOffset, VisionConstants.targetingFrontBackTranslationOffset),
                Rotation2d.fromDegrees(0)))));

        NamedCommands.registerCommand("TargetReefRight", new TargetCMD(
            visionSubsystem, drivetrain,
            driverController,
            VisionConstants.targetingRightTranslationOffset).until(() -> visionSubsystem.isNearPoseFrontTargetSpace(new Pose2d(new Translation2d(
                VisionConstants.targetingRightTranslationOffset, VisionConstants.targetingFrontBackTranslationOffset),
                Rotation2d.fromDegrees(0)))));

        NamedCommands.registerCommand("BackUp2Inches",
          drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(0.0)
            .withVelocityX(-1.0).withRotationalRate(0.0))
            .withTimeout(0.25)); // TRAIF -- will this work?

        //Named autos here
        autoChooser = AutoBuilder.buildAutoChooser("NAME SELECTED AUTO HERE");
        
        NamedCommands.registerCommand("raise elevator to L1", new ElevatorToL1CMD(elevatorSubsystem));
        NamedCommands.registerCommand("score L1", new PlaceCMD(placerSubsystem, PlacerConstants.placerFrontMotorSpeed, PlacerConstants.placerBackMotorSpeed));
        NamedCommands.registerCommand("lower elevator to home", new ElevatorToHomeCMD(elevatorSubsystem));

        SmartDashboard.putData(autoChooser);

        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(blinkinSubsystem);
        SmartDashboard.putData(climbSubsystem);
        SmartDashboard.putData(elevatorSubsystem);
        SmartDashboard.putData(hopperSubsystem);
        SmartDashboard.putData(placerSubsystem);
        SmartDashboard.putData(visionSubsystem);

        configureBindings();
    }

    private void configureBindings() {

/*
* __________________________________ DRIVER CONTROLLER ________________________________
*/

        // Run Drivetrain SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        /*driverController.povLeft().onTrue(Commands.runOnce(SignalLogger::start));
        driverController.povRight().onTrue(Commands.runOnce(SignalLogger::stop));

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // fast drive CMD
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                driveFieldCentric.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // slow drive CMD
        driverController.leftBumper().whileTrue(
            drivetrain.applyRequest(() ->
            slowDriveFieldCentric.withVelocityX(-driverController.getLeftY() * slowDriveSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * slowDriveSpeed) // Drive left with negative X (left)
                .withRotationalRate(-driverController.getRightX() * slowAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        // x lock
        driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // point wheels in a specific direction
        driverController.b().whileTrue(drivetrain.applyRequest(
            () -> point.withModuleDirection(
                new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        )); 
        
        // driverController.leftTrigger().whileTrue(
        //     new TargetCMD(visionSubsystem, drivetrain,
        //     driverController,
        //         VisionConstants.targetingLeftTranslationOffset));

        // driverController.rightTrigger().whileTrue(
        //     new TargetCMD(visionSubsystem, drivetrain,
        //     driverController,
        //         VisionConstants.targetingRightTranslationOffset));

        driverController.leftTrigger().whileTrue(
            new TargetWithLEDs(visionSubsystem, drivetrain, 
            blinkinSubsystem, driverController,
            operatorController,
            VisionConstants.targetingLeftTranslationOffset)
        );

        driverController.rightTrigger().whileTrue(
            new TargetWithLEDs(visionSubsystem, drivetrain,
            blinkinSubsystem, driverController,
            operatorController,
            VisionConstants.targetingRightTranslationOffset)
        );

        // reset the field-centric heading on start press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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

        operatorController.b().whileTrue(new ElevatorIncrementCMD(elevatorSubsystem, 3/2));
        operatorController.x().whileTrue(new ElevatorIncrementCMD(elevatorSubsystem, -3/2));

        operatorController.y().whileTrue(new ElevatorToHomeCMD(elevatorSubsystem));
        operatorController.povDown().whileTrue(new ElevatorToL1CMD(elevatorSubsystem));
        operatorController.povRight().whileTrue(new ElevatorToL2CMD(elevatorSubsystem));
        operatorController.povLeft().whileTrue(new ElevatorToL3CMD(elevatorSubsystem));
        operatorController.povUp().whileTrue(new ElevatorToL4CMD(elevatorSubsystem));

        operatorController.leftBumper().whileTrue(new ClimbOutCMD(climbSubsystem));
        operatorController.rightBumper().whileTrue(new ClimbInCMD(climbSubsystem));

        operatorController.start().whileTrue(Commands.run(
            () -> blinkinSubsystem.setPattern(
                LEDPattern.ALERT_HUMAN_PLAYER), blinkinSubsystem));

        blinkinSubsystem.setDefaultCommand(Commands.runOnce(
            () -> blinkinSubsystem.setPattern(
                LEDPattern.IDLE), blinkinSubsystem));

        placerSubsystem.setDefaultCommand(Commands.run(
            () -> placerSubsystem.setFrontMotor(
                MathUtil.applyDeadband(-operatorController.getRightY() * .25, .1)),
                placerSubsystem)
        );

        // new Trigger(placerSubsystem::isCoralInPlacer).whileTrue(Commands.run(
        //     () -> blinkinSubsystem.setPattern(LEDPattern.CORAL_INDEXED), blinkinSubsystem));

        operatorController.a().whileTrue(new CollectFromHopperCMD(placerSubsystem));

        operatorController.rightTrigger().whileTrue
        (
            new PlaceCMD(placerSubsystem, PlacerConstants.placerFrontMotorSpeed, PlacerConstants.placerBackMotorSpeed)
                //.andThen(Commands.waitSeconds(0.75))
                //.andThen(new ElevatorToHomeCMD(elevatorSubsystem))
                );
    

        // maybe add back??
        // new Trigger(hopperSubsystem::isCoralCollected).onTrue(
        //      new CollectFromHopperCMD(placerSubsystem));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
       //return Commands.print("No autonomous command configured");
    }
}
