// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PlacerCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Placer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlacerStopBothCMD extends InstantCommand {

  Placer placer;

  /** Stops both placer motors. */
  public PlacerStopBothCMD(Placer placer) {

    this.placer = placer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(placer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    placer.stopBothMotors();
  }
}
