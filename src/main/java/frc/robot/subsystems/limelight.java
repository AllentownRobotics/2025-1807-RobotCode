// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.limlightCoordinateSystemConstants;

public class LimeLight extends SubsystemBase {
  /** Creates a new limeLight. */
  NetworkTable table;
  NetworkTableEntry Id,info,cameraSet,cameraPosition;
  double id;
  /**contains the position of the april tag in the coordinate system of the robot */
  double[] Info;
  /**empty list that is populted with values to be returned, can be used in multiple places to see how to use see the getTopDownAprilTagPosition method */
  double[] returnArray;
  String limelightName;
  double[] cameraPose = new double[6];
  double[] cameraPositionArray;
  int[] primaryAprilTagIDs = new int[22];
  int[] primaryAprilTagIDsTemporary;
  int primaryAprilTagIDsLength;
  boolean priorityAprilTagsSet = false;
  
  
  /**<p>creates a new limelight</p>*/
  public LimeLight(String hostName) {
    this.limelightName = "limelight-" + hostName;
  
    this.table = NetworkTableInstance.getDefault().getTable(limelightName);
    this.Id = table.getEntry("tid");
    this.info = table.getEntry("targetpose_robotspace");
    this.cameraSet = table.getEntry("camerapose_robotspace_set");
    this.cameraPosition = table.getEntry("camerapose_robotspace");
   
    cameraPose[limlightCoordinateSystemConstants.xPosition]=0;
    cameraPose[limlightCoordinateSystemConstants.yPosition]=0;
    cameraPose[limlightCoordinateSystemConstants.zPosition]=0;
    cameraPose[limlightCoordinateSystemConstants.pitch]=0;
    cameraPose[limlightCoordinateSystemConstants.yaw]=0;
    cameraPose[limlightCoordinateSystemConstants.roll]=0;
  }
  
  // list order: x,y,z,pitch,yaw,roll
  /**sets the Limelights X position in the robots coordinate system */
  public void setX(double xPosition){
    cameraPose[limlightCoordinateSystemConstants.yPosition]=xPosition; //x is changed to y so acessing y
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the Limelights Y position in the robots coordinate system */
  public void setY(double yPosition){
    cameraPose[limlightCoordinateSystemConstants.zPosition]=yPosition; // y is changed to z so accessing s
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the Limelights Z position in the robots coordinate system */
  public void setZ(double zPosition){
    cameraPose[limlightCoordinateSystemConstants.xPosition]=zPosition; //z is changed to x so accessing x
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the pitch of the Limelight in the robots coordinate system */
  public void setPitch(double pitch){
    cameraPose[limlightCoordinateSystemConstants.yaw]=pitch; //pitch is rot. around x axis, x is changed to y, rotating around y is yaw
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the yaw of the Limelight in the robots coordinate system */
  public void setYaw(double yaw){
    cameraPose[5]=yaw; //yaw is rot. around y axis, y is changed to z, rot. around z is roll
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the roll of the Limelight in the robots coordinate system */
  public void setRoll(double roll){
    cameraPose[limlightCoordinateSystemConstants.pitch]=roll; //roll is rot. around z axis, z is changed to x, rot. around x is pitch
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets all degrees of freedom(x,y,z,pitch,yaw,roll) of the Limelight in the robots coordinate system<p>
   * input a double array with values in this order:<p>
   * X position, Y position, Z position, Pitch, Yaw, Roll
   */
  public void setAllDegreesOfFreedom(double[] cameraPoseArray){
    setX(cameraPoseArray[limlightCoordinateSystemConstants.xPosition]);
    setY(cameraPoseArray[limlightCoordinateSystemConstants.yPosition]);
    setZ(cameraPoseArray[limlightCoordinateSystemConstants.zPosition]);
    setPitch(cameraPoseArray[limlightCoordinateSystemConstants.pitch]);
    setYaw(cameraPoseArray[limlightCoordinateSystemConstants.yaw]);
    setRoll(cameraPoseArray[limlightCoordinateSystemConstants.roll]);
  }


  
  public double getAprilTagX(){
    Info = info.getDoubleArray(new double[6]);
    return Info[limlightCoordinateSystemConstants.yPosition]; //x is changed to y
  }

  public double getAprilTagY(){
    Info = info.getDoubleArray(new double[6]);
    return Info[limlightCoordinateSystemConstants.zPosition];//y is changed to z
  }

  public double getAprilTagZ(){
    Info = info.getDoubleArray(new double[6]);
    return Info[limlightCoordinateSystemConstants.xPosition];//z is changed to x
  }

  public double getAprilTagPitch(){
    Info = info.getDoubleArray(new double[6]);
    return Info[limlightCoordinateSystemConstants.yaw];// pitch is rot. around x, x is changed to y, rot around y is yaw
  }

  public double getAprilTagYaw(){
    Info = info.getDoubleArray(new double[6]);
    return Info[limlightCoordinateSystemConstants.roll];// yaw is rot. around y, y is changed to z, rot. around z is roll
  }

  public double getAprilTagRoll(){
    Info = info.getDoubleArray(new double[6]);
    return Info[limlightCoordinateSystemConstants.pitch];// roll is rot. around z, z is changed to x, rot. around x is pitch
  }

  public double getAprilTagID(){
    id = Id.getDouble(-1.0);
    return id;
  }

  public double[] getTopDownAprilTagPosition(){
    returnArray = new double[3];
    returnArray[0] = getAprilTagX();
    returnArray[1] = getAprilTagZ();
    returnArray[2] = getAprilTagYaw();
    return returnArray;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
