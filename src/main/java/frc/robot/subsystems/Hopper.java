// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {

  private DigitalInput collectorBeamBreak;

  /** Creates a new Hopper. */
  public Hopper() {
    collectorBeamBreak = new DigitalInput(HopperConstants.hopperBeamBreakID);
  }

  public boolean collectorBeamBreakStatus() {
    return collectorBeamBreak.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beam break status", collectorBeamBreakStatus());
    // This method will be called once per scheduler run
  }
}
