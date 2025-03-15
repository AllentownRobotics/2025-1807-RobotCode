// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TargetingCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlignmentValues;
import frc.robot.commands.DrivetrainCMDs.DriveCMD;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignCMD extends SequentialCommandGroup {
  /** Creates a new AlignCMD. */
  public AlignCMD(CommandSwerveDrivetrain drivetrain, Vision vision, String alignTo, CommandXboxController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveCMD(drivetrain, controller).until(vision::canSeeAprilTag),
     new TargetingDriveCMD(drivetrain, vision, alignTo, controller).raceWith(Commands.waitSeconds(AlignmentValues.timeToTarget)));
  }
}
