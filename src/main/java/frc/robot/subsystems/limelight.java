// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class limelight extends SubsystemBase {
  String limelightName = "limelight-rear";
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
NetworkTableEntry Id = table.getEntry("tid");
  NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry info = table.getEntry("targetpose_cameraspace");

  double x,y,area;
  long id;
  double[] Info;
  double test = 0;
  public limelight() {

    

    
  }

  public void putToDash(){
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    id = Id.getInteger(0);
    Info = info.getDoubleArray(new double[6]);
    if(id>0){
      NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ledMode").setNumber(2);
    } else {
      NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ledMode").setNumber(0);
    }
    
    SmartDashboard.putNumber("x",x);
    SmartDashboard.putNumber("y",y);
    SmartDashboard.putNumber("area",area);
    SmartDashboard.putNumber("ID",id);
    SmartDashboard.putNumber("test", test);
    SmartDashboard.putNumber("x pos",Info[0]);
    SmartDashboard.putNumber("y pos",Info[1]);
    SmartDashboard.putNumber("z pos",Info[2]);
    SmartDashboard.putNumber("pitch",Info[3]);
    SmartDashboard.putNumber("yaw",Info[4]);
    SmartDashboard.putNumber("roll",Info[5]);
    
    test = test+1;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
