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
import frc.robot.commands.DrivetrainCMDs.DriveCMD;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetingDriveCMD extends Command {

  SwerveRequest.RobotCentric drive;
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;
  Vision vision;
  double xMotion;//in meters
  double zMotion;//in meters
  double yawMotion;//in degrees
  double[] allignmentMovement;

  /** Creates a new DriveCMD. */
  public TargetingDriveCMD(CommandSwerveDrivetrain drivetrain, Vision vision, String alignTo, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;

    if(alignTo == "RightCoralStation"){
      allignmentMovement = vision.getRightCoralStationAllignmentValues();
    }else if(alignTo == "LeftCoralStation"){
      allignmentMovement = vision.getLeftCoralStationAllignmentValues();
    }else if(alignTo == "RightReef"){
      allignmentMovement = vision.getRightAllignmentValues();
    }else if(alignTo == "LeftReef"){
      allignmentMovement = vision.getLeftAllignmentValues();
    } else if(alignTo == "CenterCoralStation"){
      allignmentMovement = vision.getCenterCoralStationAllignmentValues();
    }

    xMotion = allignmentMovement[0];
    zMotion = Math.abs(allignmentMovement[1]);
    yawMotion = allignmentMovement[2];
    
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
      if(vision.canSeeAprilTag()){
        drive.withVelocityX((xMotion)/allignmentValues.timeToTarget)
        .withVelocityY((zMotion)/allignmentValues.timeToTarget)
        .withRotationalRate((yawMotion*Math.PI/180)/allignmentValues.timeToTarget);
      } else {
        DriveCMD driveCMD = new DriveCMD(drivetrain, controller);
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
