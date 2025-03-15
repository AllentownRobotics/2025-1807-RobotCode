// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PlacerConstants;
import frc.robot.commands.ElevatorCMDs.ElevatorToHomeCMD;
import frc.robot.commands.PlacerCMDs.PlaceCMD;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Placer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceAndElevatorToHome extends SequentialCommandGroup {
  Placer placer;
  Elevator elevator;

  /** Creates a new PlaceAndElevatorToHome. */
  public PlaceAndElevatorToHome() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PlaceCMD(placer, PlacerConstants.placerFrontMotorSpeed, PlacerConstants.placerBackMotorSpeed),
      Commands.waitUntil(placer::isCoralInPlacer),
      Commands.waitSeconds(1),
      new ElevatorToHomeCMD(elevator)
    );
  }
}
