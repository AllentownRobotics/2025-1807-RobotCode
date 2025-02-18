// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  LimeLight frontMiddleLimeLight = new LimeLight("front");
  LimeLight hopperLimeLight = new LimeLight("hopper");
  LimeLight backViewLimeLight= new LimeLight("back");
  /** Creates a new Vision System. */
  public Vision() {
    frontMiddleLimeLight.setX(0.0041);
    frontMiddleLimeLight.setY(0.158166);
    frontMiddleLimeLight.setZ(0.2579);
    frontMiddleLimeLight.setPitch(20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
