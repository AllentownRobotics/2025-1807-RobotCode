// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.TunerConstants;
import frc.robot.Constants.allignmentValues;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetingDriveCMD extends Command {

  SwerveRequest.RobotCentric drive;
  CommandSwerveDrivetrain drivetrain;
  double xMotion;//in meters
  double zMotion;//in meters
  double yawMotion;//in degrees

  /** Creates a new DriveCMD. */
  public TargetingDriveCMD(CommandSwerveDrivetrain drivetrain, double xMotion, double zMotion, double yawMotion) {
    this.drivetrain = drivetrain;
    this.xMotion = xMotion;
    this.zMotion = zMotion;
    this.yawMotion = yawMotion;
    
    drive = new SwerveRequest.RobotCentric()
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
      drive.withVelocityX((xMotion)/allignmentValues.timeToTarget)
      .withVelocityY((zMotion)/allignmentValues.timeToTarget)
      .withRotationalRate((yawMotion*Math.PI/180)/allignmentValues.timeToTarget);

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
