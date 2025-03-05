// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.allignmentValues;
import frc.robot.Constants.telementeryValues;

public class Vision extends SubsystemBase {
  /**telemetry system from the drive train */
  Telemetry telemetry = new Telemetry(telementeryValues.MaxSpeed);
  /**limelight on the front of the elevator, used to allign with the reef*/
  LimeLight frontMiddleLimeLight = new LimeLight("front");
  /** limelight on top of the hopper, facing the same firection as the front limelight to allign with the reef*/
  LimeLight hopperLimeLight = new LimeLight("hopper");
  /**limelight on the back of the robot, <p> used to allign with coral station during autonomous */
  LimeLight backViewLimeLight= new LimeLight("back");
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
  double[] targetingAllignmentReturn = new double[3];
  /** Creates a new Vision System. */
  public Vision() {
    frontMiddleLimeLight.setX(0.0041);
    frontMiddleLimeLight.setY(0.158166);
    frontMiddleLimeLight.setZ(0.2579);
    frontMiddleLimeLight.setPitch( 63);
    hopperLimeLight.setYaw(5);
    hopperLimeLight.setX(0.27305);
    hopperLimeLight.setY(0.702818);
    hopperLimeLight.setZ(0.047498);
    hopperLimeLight.setPitch(-29);

    backViewLimeLight.setZ(-0.150368);
    backViewLimeLight.setY(-0.997458);
    backViewLimeLight.setPitch(52);
    backViewLimeLight.setYaw(180);//back limelight is facing backwards, so turning it around
  }

  private void resetTemporaryTelemetry(){
    temporaryTelemetryReset = telemetry.m_poseArray;
    temporaryTelemetry[0] = 0;
    temporaryTelemetry[1] = 0;
    temporaryTelemetry[2] = 0;
  }

  private void resetAprilTagTelemetry(String limeLightName){
    resetTemporaryTelemetry();
    if(limeLightName == "front"){
      aprilTagTelemetryReset = frontMiddleLimeLight.getTopDownAprilTagPosition();
    }else if(limeLightName == "hopper"){
      aprilTagTelemetryReset = hopperLimeLight.getTopDownAprilTagPosition();
    }else if(limeLightName == "back"){
      aprilTagTelemetryReset = backViewLimeLight.getTopDownAprilTagPosition();
    }
  }

  /**boolean that can be controlled by operator  to switch the cameras to the rear cameras for climb*/
  boolean manualOverride = false;

  public void manualOverrideCamera(){
    manualOverride = !manualOverride;
  }

  private void resetAprilTagPosition(){
    if(!manualOverride){
     if(frontMiddleLimeLight.getAprilTagID() > -1 && containsCoral.robotContainsCoral()){
        //if the front limelight sees an aprilTag and the robot contains coral, reset the telemetry to the front limelight sees
        resetAprilTagTelemetry("front");
      } 
     if(hopperLimeLight.getAprilTagID() > -1 && containsCoral.robotContainsCoral()){
        //if the hopper limelight sees an aprilTag and the robot contains coral, reset the telemetry to what the the hopper limelight sees
        resetAprilTagTelemetry("hopper");
      } 
      if(backViewLimeLight.getAprilTagID() > -1 && !containsCoral.robotContainsCoral()){
        //if the back limelight sees an aprilTag and the robot does not contain coral, reset the telemetry to what the back limelight sees
        resetAprilTagTelemetry("back");
      }
    } else {
      //if the manual override is enabled, reset the telemetry to what the back limelight sees
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

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the left rod of the reef.<p>
   * Returns a double array containing the x, z, and yaw translations needed to allign with left rod.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getLeftAllignmentValues(){
    //reset the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populates the targetAllignmentReturn with the values that the robot has to move to be alligned to the left rod on the reef
    targetingAllignmentReturn[0] = allignmentValues.leftRodAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.leftRodAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.leftRodAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the right rod of the reef.<p>
   * Returns a double array containing the x, z, and yaw translations needed to allign with right rod.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getRightAllignmentValues(){
    //resets the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populated the targetAllignmentReturn with the values that the robot has to move to be alligned to the right rod of the reef
    targetingAllignmentReturn[0] = allignmentValues.rightRodAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.rightRodAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.rightRodAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the left coral station.<p>
   * Returns a double array containing the x, z, and yaw translations needed to allign with left coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getLeftCoralStationAllignmentValues(){
    //resets the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populated the targetAllignmentReturn with the values that the robot has to move to be alligned to the left side of the coral station
    targetingAllignmentReturn[0] = allignmentValues.leftCoralStationAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.leftCoralStationAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.leftCoralStationAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the center coral station.<p>
   * Returns a double array containing the x, z, and yaw translations needed to allign with center coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getCenterCoralStationAllignmentValues(){
    //resets the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populated the targetAllignmentReturn with the values that the robot has to move to be alligned to the center of the coral station
    targetingAllignmentReturn[0] = allignmentValues.centerCoralStationAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.centerCoralStationAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.centerCoralStationAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the right coral station.<p>
   * Returns a double array containing the x, z, and yaw translations needed to allign with right coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getRightCoralStationAllignmentValues(){
    //resets the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //populated the targetAllignmentReturn with the values that the robot has to move to be alligned to the right side of the coral station
    targetingAllignmentReturn[0] = allignmentValues.rightCoralStationAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.rightCoralStationAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.rightCoralStationAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**used for determining if the x,z, and yaw are alligned and if the robot itself is alligned as a whole */
  boolean allignedX, allignedZ, allignedYaw, allignedFull = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //reset the position of the last seen aprilTag so that if it currently sees an aprilTag, it will use this information instead of the last seen aprilTag
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    //determine if the robot is alligned in the x, z, and yaw directions
    if(Math.abs(getLeftAllignmentValues()[0]) < 0.1016 || Math.abs(getRightAllignmentValues()[0]) < 0.1016){
      allignedX = true;
    }
    if(Math.abs(getLeftAllignmentValues()[1]) < 0.1016 || Math.abs(getRightAllignmentValues()[1]) < 0.1016){
      allignedZ = true;
    }
    if(Math.abs(getLeftAllignmentValues()[2]) < 2 || Math.abs(getRightAllignmentValues()[2]) < 2){
      allignedYaw = true;
    }
    if(allignedX && allignedZ && allignedYaw){
      allignedFull = true;
    }

    SmartDashboard.putBoolean("Alligned", allignedFull);
    displayAprilTagTelemetry();
  }
}
