// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DrivetrainCMDs;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.TunerConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCMD extends Command {

  SwerveRequest.FieldCentric drive;
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;

  /** Creates a new DriveCMD. */
  public DriveCMD(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;

    drive = new SwerveRequest.FieldCentric()
    .withDeadband(TunerConstants.maxDriveSpeed * 0.1).withRotationalDeadband(TunerConstants.maxDriveAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrain.isSlowModeEnabled())
    {
      drive.withVelocityX(controller.getLeftX() * TunerConstants.slowDriveSpeed)
      .withVelocityY(controller.getLeftY() * TunerConstants.slowDriveSpeed)
      .withRotationalRate(controller.getRightX() * TunerConstants.slowDriveAngularRate);
    } else {
      drive.withVelocityX(controller.getLeftX() * TunerConstants.maxDriveSpeed)
      .withVelocityY(controller.getLeftY() * TunerConstants.maxDriveSpeed)
      .withRotationalRate(controller.getRightX() * TunerConstants.maxDriveAngularRate);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
