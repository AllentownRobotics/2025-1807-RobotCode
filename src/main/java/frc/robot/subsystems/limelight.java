// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.limelightCoordinateSystemConstants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTable table;
  NetworkTableEntry AprilTagId, infoAboutAprilTagDegreesOfFreedom, cameraSet, cameraPosition, LimelightLED;
  /**Primary in view AprilTag ID */
  double aprilTagId;
  /**contains the position of the april tag in the coordinate system of the robot, 
   * the order of the values is:<p>
   * X position, Y position, Z position, Pitch, Yaw, Roll
   */
  double[] InfoAboutAprilTagDegreesOfFreedom;
  /**empty list that is populted with values to be returned, can be used in multiple places to see how to use see the getTopDownAprilTagPosition method */
  double[] returnArray;
  String LimelightName;
  double[] cameraPose = new double[6];
  double[] cameraPositionArray;
  int[] primaryAprilTagIDs = new int[22];
  int[] primaryAprilTagIDsTemporary;
  int primaryAprilTagIDsLength;
  boolean priorityAprilTagsSet = false;
  
  
  /**<p>creates a new Limelight</p>*/
  public Limelight(String hostName) {
    this.LimelightName = "Limelight-" + hostName;
  
    this.table = NetworkTableInstance.getDefault().getTable(LimelightName);
    this.AprilTagId = table.getEntry("tid");
    this.infoAboutAprilTagDegreesOfFreedom = table.getEntry("targetpose_robotspace");
    this.cameraSet = table.getEntry("camerapose_robotspace_set");
    this.cameraPosition = table.getEntry("camerapose_robotspace");
    this.LimelightLED = table.getEntry("ledMode");
   
    cameraPose[limelightCoordinateSystemConstants.xPosition]=0;
    cameraPose[limelightCoordinateSystemConstants.yPosition]=0;
    cameraPose[limelightCoordinateSystemConstants.zPosition]=0;
    cameraPose[limelightCoordinateSystemConstants.pitch]=0;
    cameraPose[limelightCoordinateSystemConstants.yaw]=0;
    cameraPose[limelightCoordinateSystemConstants.roll]=0;
  }
  
  // list order: x,y,z,pitch,yaw,roll
  /**sets the Limelights X position in the robots coordinate system */
  public void setX(double xPosition){
    cameraPose[limelightCoordinateSystemConstants.yPosition]=xPosition; //x is changed to y so acessing y
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the Limelights Y position in the robots coordinate system */
  public void setY(double yPosition){
    cameraPose[limelightCoordinateSystemConstants.zPosition]=yPosition; // y is changed to z so accessing s
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the Limelights Z position in the robots coordinate system */
  public void setZ(double zPosition){
    cameraPose[limelightCoordinateSystemConstants.xPosition]=zPosition; //z is changed to x so accessing x
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the pitch of the Limelight in the robots coordinate system */
  public void setPitch(double pitch){
    cameraPose[limelightCoordinateSystemConstants.yaw]=pitch; //pitch is rot. around x axis, x is changed to y, rotating around y is yaw
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the yaw of the Limelight in the robots coordinate system */
  public void setYaw(double yaw){
    cameraPose[5]=yaw; //yaw is rot. around y axis, y is changed to z, rot. around z is roll
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets the roll of the Limelight in the robots coordinate system */
  public void setRoll(double roll){
    cameraPose[limelightCoordinateSystemConstants.pitch]=roll; //roll is rot. around z axis, z is changed to x, rot. around x is pitch
    cameraSet.setDoubleArray(cameraPose);
  }

  /**sets all degrees of freedom(x,y,z,pitch,yaw,roll) of the Limelight in the robots coordinate system<p>
   * input a double array with values in this order:<p>
   * X position, Y position, Z position, Pitch, Yaw, Roll
   */
  public void setAllDegreesOfFreedom(double[] cameraPoseArray){
    setX(cameraPoseArray[limelightCoordinateSystemConstants.xPosition]);
    setY(cameraPoseArray[limelightCoordinateSystemConstants.yPosition]);
    setZ(cameraPoseArray[limelightCoordinateSystemConstants.zPosition]);
    setPitch(cameraPoseArray[limelightCoordinateSystemConstants.pitch]);
    setYaw(cameraPoseArray[limelightCoordinateSystemConstants.yaw]);
    setRoll(cameraPoseArray[limelightCoordinateSystemConstants.roll]);
  }

  public void turnOnLED(){
    LimelightLED.setDouble(3);
  }
  
  public void turnOffLED(){
    LimelightLED.setDouble(1);
  }

  public void blinkLED(){
    LimelightLED.setDouble(2);
  }

  public void defaultLED(){
    LimelightLED.setDouble(0);
  }


  
  public double getAprilTagX(){
    InfoAboutAprilTagDegreesOfFreedom = infoAboutAprilTagDegreesOfFreedom.getDoubleArray(new double[6]);
    return InfoAboutAprilTagDegreesOfFreedom[limelightCoordinateSystemConstants.yPosition]; //x is changed to y
  }

  public double getAprilTagY(){
    InfoAboutAprilTagDegreesOfFreedom = infoAboutAprilTagDegreesOfFreedom.getDoubleArray(new double[6]);
    return InfoAboutAprilTagDegreesOfFreedom[limelightCoordinateSystemConstants.zPosition];//y is changed to z
  }

  public double getAprilTagZ(){
    InfoAboutAprilTagDegreesOfFreedom = infoAboutAprilTagDegreesOfFreedom.getDoubleArray(new double[6]);
    return InfoAboutAprilTagDegreesOfFreedom[limelightCoordinateSystemConstants.xPosition];//z is changed to x
  }

  public double getAprilTagPitch(){
    InfoAboutAprilTagDegreesOfFreedom = infoAboutAprilTagDegreesOfFreedom.getDoubleArray(new double[6]);
    return InfoAboutAprilTagDegreesOfFreedom[limelightCoordinateSystemConstants.yaw];// pitch is rot. around x, x is changed to y, rot around y is yaw
  }

  public double getAprilTagYaw(){
    InfoAboutAprilTagDegreesOfFreedom = infoAboutAprilTagDegreesOfFreedom.getDoubleArray(new double[6]);
    return InfoAboutAprilTagDegreesOfFreedom[limelightCoordinateSystemConstants.roll];// yaw is rot. around y, y is changed to z, rot. around z is roll
  }

  public double getAprilTagRoll(){
    InfoAboutAprilTagDegreesOfFreedom = infoAboutAprilTagDegreesOfFreedom.getDoubleArray(new double[6]);
    return InfoAboutAprilTagDegreesOfFreedom[limelightCoordinateSystemConstants.pitch];// roll is rot. around z, z is changed to x, rot. around x is pitch
  }

  public double getAprilTagID(){
    aprilTagId = AprilTagId.getDouble(-1.0);//-1.0 is returned if no AprilTag is found because 0 is a valid id, no id is -1
    return aprilTagId;
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
