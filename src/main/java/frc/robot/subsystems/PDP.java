// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDP extends SubsystemBase {
  PowerDistribution pdp;
  int i;
  /** Creates a new PowerDistribution. */
  public PDP() {
    pdp = new PowerDistribution(0, ModuleType.kRev);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putData("PDP", new Sendable() {
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Voltage", () -> pdp.getVoltage(), null);
    builder.addDoubleProperty("Total Current", () -> pdp.getTotalCurrent(), null);


    for(i=0; i < 24; i++)
    {
      String chan = "Ch " + i;
      builder.addDoubleProperty(chan, () -> pdp.getCurrent(i), null);
    }
  
  }
});
    // This method will be called once per scheduler run
  }
}
