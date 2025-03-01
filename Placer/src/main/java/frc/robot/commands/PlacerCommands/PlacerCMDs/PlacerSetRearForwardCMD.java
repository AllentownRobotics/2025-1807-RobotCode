// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:Placer/src/main/java/frc/robot/commands/PlacerCommands/PlacerCMDs/PlacerSetRearForwardCMD.java
package frc.robot.commands.PlacerCMDs;
========
package frc.robot.commands.PlacerCommands;
>>>>>>>> remy_placer:Placer/src/main/java/frc/robot/commands/PlacerCommands/PlacerSetRearReverseCMD.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Placer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
<<<<<<<< HEAD:Placer/src/main/java/frc/robot/commands/PlacerCommands/PlacerCMDs/PlacerSetRearForwardCMD.java
public class PlacerSetRearForwardCMD extends Command {

  Placer placer;
  double speed;

  /** Sets the speed of the front placer motor. */
  public PlacerSetRearForwardCMD(Placer placer, double speed) {
    this.placer = placer;
    this.speed = speed;

========
public class PlacerSetRearReverseCMD extends Command {

  Placer placer;
  double speed;

  /** Sets the speed of the front placer motor. */
  public PlacerSetRearReverseCMD(Placer placer, double speed) {

    this.placer = placer;
    this.speed = speed;
>>>>>>>> remy_placer:Placer/src/main/java/frc/robot/commands/PlacerCommands/PlacerSetRearReverseCMD.java
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(placer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<<< HEAD:Placer/src/main/java/frc/robot/commands/PlacerCommands/PlacerCMDs/PlacerSetRearForwardCMD.java
    placer.setRearMotorPlus(speed);
========
    placer.setRearMotorReverse(speed);
>>>>>>>> remy_placer:Placer/src/main/java/frc/robot/commands/PlacerCommands/PlacerSetRearReverseCMD.java
  }

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
