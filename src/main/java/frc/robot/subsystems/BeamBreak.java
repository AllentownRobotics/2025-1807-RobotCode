// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreak extends SubsystemBase {
  private DigitalInput collectorBeamBreak;
  /** Creates a new BeamBreak. */
  public BeamBreak() {
    collectorBeamBreak = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    isBeamBreakBroken();
    // This method will be called once per scheduler run
  }

  public boolean isCollectorBeamBreak() {
    return !collectorBeamBreak.get();
  }
  public void isBeamBreakBroken() {
    SmartDashboard.putBoolean("beam break", isCollectorBeamBreak());
  }
}
