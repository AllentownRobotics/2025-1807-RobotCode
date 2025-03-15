// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag; // Replace 'some.package' with the actual package name
public class limelight extends SubsystemBase {
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-rear");
NetworkTableEntry Id = table.getEntry("tid");
  NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry info = table.getEntry("targetpose_cameraspace");

  static double x,y,area;
  static long id;
  static double[] Info;
  double test = 0;
  public limelight() {

    

    
  }
  public double getInfo(int item){
    Info = info.getDoubleArray(new double[6]);
    return Info[item];
  }

  public void putToDash(){
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    id = Id.getInteger(0);
    Info = info.getDoubleArray(new double[6]);
    
    SmartDashboard.putNumber("x",x);
    SmartDashboard.putNumber("y",y);
    SmartDashboard.putNumber("area",area);
    SmartDashboard.putNumber("ID",id);
    SmartDashboard.putNumber("test", test);
    SmartDashboard.putNumberArray("info",Info);
    
    test = test+1;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
