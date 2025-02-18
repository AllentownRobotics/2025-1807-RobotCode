// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  /** Creates a new limeLight. */
  NetworkTable table;
  NetworkTableEntry Id,info,cameraSet,cameraPosition;
  double id;
  double[] Info;
  double[] targetInfo;
  String limelightName;
  double[] cameraPose = new double[6];
  double[] cameraPositionArray;
  
  
  /**<p>creates a new limelight</p>*/
  public LimeLight(String hostName) {
    this.limelightName = "limelight-" + hostName;
  
    this.table = NetworkTableInstance.getDefault().getTable(limelightName);
    this.Id = table.getEntry("tid");
    this.info = table.getEntry("targetpose_robotspace");
    this.cameraSet = table.getEntry("camerapose_robotspace_set");
    this.cameraPosition = table.getEntry("camerapose_robotspace");
   
    cameraPose[0]=0;
    cameraPose[1]=0;
    cameraPose[2]=0;
    cameraPose[3]=0;
    cameraPose[4]=0;
    cameraPose[5]=0;
  }
  
  // list order: x,y,z,pitch,yaw,roll
  public void setX(double xPosition){
    cameraPose[1]=xPosition; //x is changed to y so acessing y
    cameraSet.setDoubleArray(cameraPose);
  }

  public void setY(double yPosition){
    cameraPose[2]=yPosition; // y is changed to z so accessing s
    cameraSet.setDoubleArray(cameraPose);
  }

  public void setZ(double zPosition){
    cameraPose[0]=zPosition; //z is changed to x so accessing x
    cameraSet.setDoubleArray(cameraPose);
  }

  public void setPitch(double pitch){
    cameraPose[4]=pitch; //pitch is rot. around x axis, x is changed to y, rotating around y is yaw
    cameraSet.setDoubleArray(cameraPose);
  }

  public void setYaw(double yaw){
    cameraPose[5]=yaw; //yaw is rot. around y axis, y is changed to z, rot. around z is roll
    cameraSet.setDoubleArray(cameraPose);
  }

  public void setRoll(double roll){
    cameraPose[3]=roll; //roll is rot. around z axis, z is changed to x, rot. around x is pitch
    cameraSet.setDoubleArray(cameraPose);
  }

  public void setAllDegreesOfFreedom(double[] cameraPoseArray){
    cameraPose = cameraPoseArray;
    cameraSet.setDoubleArray(cameraPose);
  }

  public double getAprilTagX(){
    Info = info.getDoubleArray(new double[6]);
    return Info[1]; //x is changed to y
  }

  public double getAprilTagY(){
    Info = info.getDoubleArray(new double[6]);
    return Info[2];//y is changed to z
  }

  public double getAprilTagZ(){
    Info = info.getDoubleArray(new double[6]);
    return Info[0];//z is changed to x
  }

  public double getAprilTagPitch(){
    Info = info.getDoubleArray(new double[6]);
    return Info[4];// pitch is rot. around x, x is changed to y, rot around y is yaw
  }

  public double getAprilTagYaw(){
    Info = info.getDoubleArray(new double[6]);
    return Info[5];// yaw is rot. around y, y is changed to z, rot. around z is roll
  }

  public double getAprilTagRoll(){
    Info = info.getDoubleArray(new double[6]);
    return Info[3];// roll is rot. around z, z is changed to x, rot. around x is pitch
  }

  public double getAprilTagID(){
    id = Id.getDouble(0.0);
    return id;
  }

  public void putCameraPose(){
      cameraPositionArray = cameraPosition.getDoubleArray(new double[6]);
      SmartDashboard.putNumber("x pos",cameraPositionArray[0]);
      SmartDashboard.putNumber("y pos",cameraPositionArray[1]);
      SmartDashboard.putNumber("z pos",cameraPositionArray[2]);
      SmartDashboard.putNumber("pitch",cameraPositionArray[3]);
      SmartDashboard.putNumber("yaw",cameraPositionArray[4]);
      SmartDashboard.putNumber("roll",cameraPositionArray[5]);
  }

  public double[] getAprilTagPosition(){
    id = Id.getDouble(0.0);
    Info = info.getDoubleArray(new double[6]);
    targetInfo = new double[]{id,Info[0],Info[1],Info[2],Info[3],Info[4],Info[5]};
    SmartDashboard.putNumber("camera x pos", table.getEntry("camerapose_robotspace").getDoubleArray(new double[6])[0]);
    /**SmartDashboard.putNumber("x",x);
    SmartDashboard.putNumber("y",y);
    SmartDashboard.putNumber("area",area);
    SmartDashboard.putNumber("ID",id);
    SmartDashboard.putNumber("x pos",Info[0]);
    SmartDashboard.putNumber("y pos",Info[1]);
    SmartDashboard.putNumber("z pos",Info[2]);
    SmartDashboard.putNumber("pitch",Info[3]);
    SmartDashboard.putNumber("yaw",Info[4]);
    SmartDashboard.putNumber("roll",Info[5]);*/
    return targetInfo;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
