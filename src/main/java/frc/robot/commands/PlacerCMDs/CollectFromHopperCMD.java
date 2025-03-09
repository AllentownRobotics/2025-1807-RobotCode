// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlacerCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PlacerConstants;
import frc.robot.subsystems.Placer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectFromHopperCMD extends Command {
  Placer placer;
  /** Creates a new CollectFromHopper. */
  public CollectFromHopperCMD(Placer placer) {
    this.placer = placer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(placer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    placer.setRearMotor(PlacerConstants.placerBackMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    placer.setRearMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return placer.isCoralInPlacer();
  
  }
}
