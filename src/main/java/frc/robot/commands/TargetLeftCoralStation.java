// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetLeftCoralStation extends Command {
  Vision vision;
  //DriveTrain drivetrain;
  double xAllignmentValue;
  double zAllignmentValue;
  double yawAllignmentValue;
  /** Creates a new TargetLeftCoralStation. */
  public TargetLeftCoralStation(Vision vision/*,DriveTrain driveTrain */) {
    this.vision = vision;
    //this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision/*,driveTrain*/);
  }

  public void target(){
    xAllignmentValue = vision.getLeftCoralStationAllignmentValues()[0];
    zAllignmentValue = vision.getLeftCoralStationAllignmentValues()[1];
    yawAllignmentValue = vision.getLeftCoralStationAllignmentValues()[2];
    //driveTrain.drive(xAllignmentValue, zAllignmentValue, yawAllignmentValue);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
