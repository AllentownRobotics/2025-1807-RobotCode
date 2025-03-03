// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  double x;
  double y;
  double a;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  NetworkTable table;
  
  /** Creates a new Limelight. */
  public Limelight() {
    
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  public double getX() {
    return x;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);

    SmartDashboard.putNumber("x value", x);
    SmartDashboard.putNumber("y value", y);
    SmartDashboard.putNumber("area", a);
  }
}
