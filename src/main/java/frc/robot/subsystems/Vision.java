// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.allignmentValues;
import frc.robot.Constants.telementeryValues;

public class Vision extends SubsystemBase {
  Telemetry telemetry = new Telemetry(telementeryValues.MaxSpeed);
  LimeLight frontMiddleLimeLight = new LimeLight("front");
  LimeLight hopperLimeLight = new LimeLight("hopper");
  LimeLight backViewLimeLight= new LimeLight("back");
  RobotCoralState containsCoral = new RobotCoralState();

  double[] temporaryTelemetryReset = new double[3];
  double[] temporaryTelemetry = new double[3];
  double[] aprilTagTelemetry = new double[3];
  double[] aprilTagTelemetryReset = new double[3];

  double[] targetingAllignmentReturn = new double[3];
  /** Creates a new Vision System. */
  public Vision() {
    frontMiddleLimeLight.setX(0.0041);
    frontMiddleLimeLight.setY(0.158166);
    frontMiddleLimeLight.setZ(0.2579);
    frontMiddleLimeLight.setPitch(20);
    hopperLimeLight.setYaw(10);
    hopperLimeLight.setPitch(331);
  }

  private void resetTemporaryTelementery(){
    temporaryTelemetryReset = telemetry.m_poseArray;
    temporaryTelemetry[0] = 0;
    temporaryTelemetry[1] = 0;
    temporaryTelemetry[2] = 0;
  }

  private void resetAprilTagTelemetry(String limeLightName){
    resetTemporaryTelementery();
    if(limeLightName == "front"){
      aprilTagTelemetryReset = frontMiddleLimeLight.getTopDownAprilTagPosition();
    }else if(limeLightName == "hopper"){
      aprilTagTelemetryReset = hopperLimeLight.getTopDownAprilTagPosition();
    }else if(limeLightName == "back"){
      aprilTagTelemetryReset = backViewLimeLight.getTopDownAprilTagPosition();
    }
  }

  boolean manualOverride = false;

  public void manualOverrideCamera(){
    manualOverride = !manualOverride;
  }

  private void resetAprilTagPosition(){
    if(!manualOverride){
     if(frontMiddleLimeLight.getAprilTagID() > -1 && containsCoral.robotContainsCoral()){
        resetAprilTagTelemetry("front");
      } 
     if(hopperLimeLight.getAprilTagID() > -1 && containsCoral.robotContainsCoral()){
        resetAprilTagTelemetry("hopper");
      } 
      if(backViewLimeLight.getAprilTagID() > -1 && !containsCoral.robotContainsCoral()){
        resetAprilTagTelemetry("back");
      }
    } else {
      resetAprilTagTelemetry("back");
    }
  }
  //m_poseArray is the current position of the robot in the field coordinate system.
  //temporaryTelemetryReset is the refrence point of the last time the robot saw a aprilTag in the fields coordinate system.
  private void setTemporaryTelementery(){
    temporaryTelemetry[0] = telemetry.m_poseArray[0] - temporaryTelemetryReset[0];
    temporaryTelemetry[1] = telemetry.m_poseArray[1] - temporaryTelemetryReset[1];
    temporaryTelemetry[2] = telemetry.m_poseArray[2] - temporaryTelemetryReset[2];
  }

  private void setAprilTagTelemetry(){
    aprilTagTelemetry[0] = temporaryTelemetry[0] - aprilTagTelemetryReset[0];
    aprilTagTelemetry[1] = temporaryTelemetry[1] - aprilTagTelemetryReset[1];
    aprilTagTelemetry[2] = temporaryTelemetry[2] - aprilTagTelemetryReset[2];
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the left rod of the reef.<p>
   * Returns a double array containing the x,, z, and yaw translations needed to allign with left rod.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getLeftAllignmentValues(){
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    targetingAllignmentReturn[0] = allignmentValues.leftRodAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.leftRodAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.leftRodAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the right rod of the reef.<p>
   * Returns a double array containing the x,, z, and yaw translations needed to allign with right rod.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getRightAllignmentValues(){
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    targetingAllignmentReturn[0] = allignmentValues.rightRodAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.rightRodAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.rightRodAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the left coral station.<p>
   * Returns a double array containing the x,, z, and yaw translations needed to allign with left coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getLeftCoralStationAllignmentValues(){
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    targetingAllignmentReturn[0] = allignmentValues.leftCoralStationAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.leftCoralStationAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.leftCoralStationAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the center coral station.<p>
   * Returns a double array containing the x,, z, and yaw translations needed to allign with center coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getCenterCoralStationAllignmentValues(){
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    targetingAllignmentReturn[0] = allignmentValues.centerCoralStationAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.centerCoralStationAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.centerCoralStationAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  /**Returns an array that contains the translations and rotation that the robot has to make to allign with the right coral station.<p>
   * Returns a double array containing the x,, z, and yaw translations needed to allign with right coral station.<p>
   * Order of array returned is: [X translation, Z translation, Yaw rotation].
   */
  public double[] getRightCoralStationAllignmentValues(){
    resetAprilTagPosition();
    setTemporaryTelementery();
    setAprilTagTelemetry();
    targetingAllignmentReturn[0] = allignmentValues.rightCoralStationAllignmentX - aprilTagTelemetry[0];
    targetingAllignmentReturn[1] = allignmentValues.rightCoralStationAllignmentZ - aprilTagTelemetry[1];
    targetingAllignmentReturn[2] = allignmentValues.rightCoralStationAllignmentYaw - aprilTagTelemetry[2];
    return targetingAllignmentReturn;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
