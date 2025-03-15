// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlignmentValues;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  /**telemetry system from the drive train */
  Telemetry telemetry = new Telemetry(RobotContainer.MaxSpeed);
  /**Limelight on the front of the elevator, used to align with the reef*/
  Limelight frontMiddleLimelight = new Limelight("front");
  /** Limelight on top of the hopper, facing the same firection as the front Limelight to align with the reef*/
  Limelight hopperLimelight = new Limelight("hopper");
  /**Limelight on the back of the robot, <p> used to align with coral station during autonomous */
  Limelight backViewLimelight= new Limelight("back");
  /**robot coral state, used to determine if the robot contains coral to determine which camera to use for targeting*/
  RobotCoralState containsCoral = new RobotCoralState();

  /**coordinates of the robot in the fields coordinate system at the last time the camera
   * (determined by the coral state in the robot) saw an aprilTag
   */
  double[] temporaryTelemetryReset = new double[3];
  /**current position in the field relative to the last position the robot saw an aprilTag */
  double[] temporaryTelemetry = new double[3];
  /**transformations to the last seen aprilTag in thecoordinate system of the robot */
  double[] aprilTagTelemetry = new double[3];
  /**position of the last seen apriltag relative to the position on the field that the robot last saw it */
  double[] aprilTagTelemetryReset = new double[3];

  /**empty list that is populated and returned when accessing the targeting methods */
  double[] targetingalignmentReturn = new double[3];
  /** Creates a new Vision System. */
  public Vision() {

// CONFIRM VALUES IN LIMELIGHT ONLINE CAMERA
    frontMiddleLimelight.setX(VisionConstants.frontLLRight);
    frontMiddleLimelight.setY(VisionConstants.frontLLForward);
    frontMiddleLimelight.setZ(VisionConstants.frontLLUp);
    frontMiddleLimelight.setPitch(VisionConstants.frontLLPitch);
    frontMiddleLimelight.setYaw(VisionConstants.frontLLYaw);
    frontMiddleLimelight.setRoll(VisionConstants.frontLLRoll);

    hopperLimelight.setX(VisionConstants.hopperLLRight);
    hopperLimelight.setY(VisionConstants.hopperLLForward);
    hopperLimelight.setZ(VisionConstants.hopperLLUp);
    hopperLimelight.setPitch(VisionConstants.hopperLLPitch);
    hopperLimelight.setYaw(VisionConstants.hopperLLYaw);
    hopperLimelight.setRoll(VisionConstants.hopperLLRoll);

    backViewLimelight.setX(VisionConstants.backLLRight);
    backViewLimelight.setY(VisionConstants.backLLForward);
    backViewLimelight.setZ(VisionConstants.backLLUp);
    backViewLimelight.setPitch(VisionConstants.backLLPitch);
    backViewLimelight.setYaw(VisionConstants.backLLYaw);//back Limelight is facing backwards, so turning it around
    backViewLimelight.setRoll(VisionConstants.backLLRoll);
  }

  private void resetTemporaryTelemetry(){
    temporaryTelemetryReset = new double[]{0,0,0};
    //temporaryTelemetryReset = telemetry.m_poseArray;
    temporaryTelemetry[0] = 0;
    temporaryTelemetry[1] = 0;
    temporaryTelemetry[2] = 0;
  }

  private void resetAprilTagTelemetry(String LimelightName){
    resetTemporaryTelemetry();
    if(LimelightName == "front"){
      aprilTagTelemetryReset = frontMiddleLimelight.getTopDownAprilTagPosition();
    }else if(LimelightName == "hopper"){
      aprilTagTelemetryReset = hopperLimelight.getTopDownAprilTagPosition();
    }else if(LimelightName == "back"){
      aprilTagTelemetryReset = backViewLimelight.getTopDownAprilTagPosition();
    }
  }

  /**boolean that can be controlled by operator  to switch the cameras to the rear cameras for climb*/
  boolean manualOverride = false;

  public void manualOverrideCamera(){
    manualOverride = !manualOverride;
  }

  private void resetAprilTagPosition(){
    if(!manualOverride){
     if(frontMiddleLimelight.getAprilTagID() > -1 && containsCoral.robotContainsCoral()){
        //if the front Limelight sees an aprilTag and the robot contains coral, reset the telemetry to the front Limelight sees
        resetAprilTagTelemetry("front");
      } else 
     if(hopperLimelight.getAprilTagID() > -1 && containsCoral.robotContainsCoral()){
        //if the hopper Limelight sees an aprilTag and the robot contains coral, reset the telemetry to what the the hopper Limelight sees
        resetAprilTagTelemetry("hopper");
      } 
      if(backViewLimelight.getAprilTagID() > -1 && !containsCoral.robotContainsCoral()){
        //if the back Limelight sees an aprilTag and the robot does not contain coral, reset the telemetry to what the back Limelight sees
        resetAprilTagTelemetry("back");
      }
    } else {
      //if the manual override is enabled, reset the telemetry to what the back Limelight sees
      resetAprilTagTelemetry("back");
    }
  }
  //m_poseArray is the current position of the robot in the field coordinate system.
  //temporaryTelemetryReset is the refrence point of the last time the robot saw a aprilTag in the fields coordinate system.
  private void setTemporaryTelementery(){
    //set the temporary telemetry to the current position of the robot relative to the last time the robot saw an aprilTag
    //determined by using the current position of the robot on the field and the refrence point of the last time the robot saw an aprilTag
    temporaryTelemetry[0] = telemetry.m_poseArray[0] - temporaryTelemetryReset[0];
    temporaryTelemetry[1] = telemetry.m_poseArray[1] - temporaryTelemetryReset[1];
    temporaryTelemetry[2] = telemetry.m_poseArray[2] - temporaryTelemetryReset[2];
  }

  private void setAprilTagTelemetry(){
    //set the aprilTagTelemetry to the current position of the robot relative to the last time the robot saw an aprilTag
    //determined by using the temporary telemetry as transformations to be done to the aprilTag telemetry refrence point
    aprilTagTelemetry[0] = temporaryTelemetry[0] - aprilTagTelemetryReset[0];
    aprilTagTelemetry[1] = temporaryTelemetry[1] - aprilTagTelemetryReset[1];
    aprilTagTelemetry[2] = temporaryTelemetry[2] - aprilTagTelemetryReset[2];
  }

  public void displayAprilTagTelemetry(){
    SmartDashboard.putNumber("aprilTagX",aprilTagTelemetry[0]);
    SmartDashboard.putNumber("aprilTagZ",aprilTagTelemetry[1]);
    SmartDashboard.putNumber("aprilTagYaw",aprilTagTelemetry[2]);
  }

  public boolean canSeeAprilTag(){
    if(frontMiddleLimelight.getAprilTagID() > -1 || hopperLimelight.getAprilTagID() > -1){
      return true;
    } else {
      return false;
    }
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to align with the left rod of the reef.<p>
   * Returns a double array containing the x, z, and yaw translations needed to align with left rod.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getLeftalignmentValues(){
    //reset the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populates the targetalignmentReturn with the values that the robot has to move to be aligned to the left rod on the reef
    targetingalignmentReturn[0] = AlignmentValues.leftRodAlignmentX - aprilTagTelemetry[0];
    targetingalignmentReturn[1] = AlignmentValues.leftRodAlignmentZ - aprilTagTelemetry[1];
    targetingalignmentReturn[2] = AlignmentValues.leftRodAlignmentYaw - aprilTagTelemetry[2];
    return targetingalignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to align with the right rod of the reef.<p>
   * Returns a double array containing the x, z, and yaw translations needed to align with right rod.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getRightalignmentValues(){
    //resets the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populated the targetalignmentReturn with the values that the robot has to move to be aligned to the right rod of the reef
    targetingalignmentReturn[0] = AlignmentValues.rightRodAlignmentX - aprilTagTelemetry[0];
    targetingalignmentReturn[1] = AlignmentValues.rightRodAlignmentZ - aprilTagTelemetry[1];
    targetingalignmentReturn[2] = AlignmentValues.rightRodAlignmentYaw - aprilTagTelemetry[2];
    return targetingalignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to align with the left coral station.<p>
   * Returns a double array containing the x, z, and yaw translations needed to align with left coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getLeftCoralStationalignmentValues(){
    //resets the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populated the targetalignmentReturn with the values that the robot has to move to be aligned to the left side of the coral station
    targetingalignmentReturn[0] = AlignmentValues.leftCoralStationAlignmentX - aprilTagTelemetry[0];
    targetingalignmentReturn[1] = AlignmentValues.leftCoralStationAlignmentZ - aprilTagTelemetry[1];
    targetingalignmentReturn[2] = AlignmentValues.leftCoralStationAlignmentYaw - aprilTagTelemetry[2];
    return targetingalignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to align with the center coral station.<p>
   * Returns a double array containing the x, z, and yaw translations needed to align with center coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getCenterCoralStationalignmentValues(){
    //resets the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populated the targetalignmentReturn with the values that the robot has to move to be aligned to the center of the coral station
    targetingalignmentReturn[0] = AlignmentValues.centerCoralStationAlignmentX - aprilTagTelemetry[0];
    targetingalignmentReturn[1] = AlignmentValues.centerCoralStationAlignmentZ - aprilTagTelemetry[1];
    targetingalignmentReturn[2] = AlignmentValues.centerCoralStationAlignmentYaw - aprilTagTelemetry[2];
    return targetingalignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to align with the right coral station.<p>
   * Returns a double array containing the x, z, and yaw translations needed to align with right coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getRightCoralStationalignmentValues(){
    //resets the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populated the targetalignmentReturn with the values that the robot has to move to be aligned to the right side of the coral station
    targetingalignmentReturn[0] = AlignmentValues.rightCoralStationAlignmentX - aprilTagTelemetry[0];
    targetingalignmentReturn[1] = AlignmentValues.rightCoralStationAlignmentZ - aprilTagTelemetry[1];
    targetingalignmentReturn[2] = AlignmentValues.rightCoralStationAlignmentYaw - aprilTagTelemetry[2];
    return targetingalignmentReturn;
  }

  /**used for determining if the x,z, and yaw are aligned and if the robot itself is aligned as a whole */
  boolean alignedX, alignedZ, alignedYaw, alignedFull = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //reset the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //determine if the robot is aligned in the x, z, and yaw directions
    if(Math.abs(getLeftalignmentValues()[0]) < 0.1016 || Math.abs(getRightalignmentValues()[0]) < 0.1016){
      alignedX = true;
    }
    if(Math.abs(getLeftalignmentValues()[1]) < 0.1016 || Math.abs(getRightalignmentValues()[1]) < 0.1016){
      alignedZ = true;
    }
    if(Math.abs(getLeftalignmentValues()[2]) < 2 || Math.abs(getRightalignmentValues()[2]) < 2){
      alignedYaw = true;
    }
    if(alignedX && alignedZ && alignedYaw){
      alignedFull = true;
    }

    SmartDashboard.putBoolean("aligned", alignedFull);
    displayAprilTagTelemetry();
    SmartDashboard.putNumber("x change for left", getLeftalignmentValues()[0]);
    SmartDashboard.putNumber("z change for left", getLeftalignmentValues()[1]);
    SmartDashboard.putNumber("yaw change for left", getLeftalignmentValues()[2]);
    SmartDashboard.putNumber("x change for right", getRightalignmentValues()[0]);
    SmartDashboard.putNumber("z change for right", getRightalignmentValues()[1]);
    SmartDashboard.putNumber("yaw change for right", getRightalignmentValues()[2]);
  }
}
