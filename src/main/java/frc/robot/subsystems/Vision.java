// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  NetworkTable frontLimelightTable;
  NetworkTable hopperLimelightTable;
  NetworkTable rearLimelightTable;

  double rightRotationOffset;
  double rightXTranslationOffset;
  double rightZTranslationOffset;
  double rightXTranslationOffsetToPlacer;

  double leftRotationOffset;
  double leftXTranslationOffset;
  double leftZTranslationOffset;
  double leftXTranslationOffsetToPlacer;

  PIDController rotationController;
  PIDController translationController;

  boolean alignedLeft = false;
  boolean alignedRight = false;

  //public boolean linedUpEnough;

  /** Creates a new Vision. */
  public Vision() {

    frontLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    hopperLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-hopper");
    rearLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-back");

    frontLimelightTable.getEntry("priorityid").setNumber(-1);
    hopperLimelightTable.getEntry("priorityid").setNumber(-1);
    rearLimelightTable.getEntry("priorityid").setNumber(-1);

    rotationController = new PIDController(VisionConstants.rotation_kP, VisionConstants.rotation_kI, VisionConstants.rotation_kD);
    translationController = new PIDController(VisionConstants.translation_kP, VisionConstants.translation_kI, VisionConstants.translation_kD);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // right side reef targeting
    rightRotationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    rightXTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
    rightZTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

    rightXTranslationOffsetToPlacer = rightXTranslationOffset + VisionConstants.placerOffsetToRobotCenter;

    /*
    if(Math.abs(rightXTranslationOffsetToPlacer) <= VisionConstants.alignmentDeadzone) {
      rightXTranslationOffsetToPlacer = 0;
    } */

    //left side reef targeting
    leftRotationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    leftXTranslationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
    leftZTranslationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

    leftXTranslationOffsetToPlacer = leftXTranslationOffset + 0.05;

    /*
    if(Math.abs(leftXTranslationOffsetToPlacer) <= VisionConstants.alignmentDeadzone) {
      leftXTranslationOffsetToPlacer = 0;
    } */

    //SmartDashboard.putNumber("Right Rotation Offset", rightRotationOffset);
    SmartDashboard.putNumber("rightXTranslationOffset", rightXTranslationOffset);
    SmartDashboard.putNumber("rightZTranslationOffset", rightZTranslationOffset);
    SmartDashboard.putNumber("rightXTranslationOffsetToPlacer", rightXTranslationOffsetToPlacer);
    //SmartDashboard.putNumber("right rotation pid output", getRightRotationPID());
    SmartDashboard.putNumber("right translation pid output", getRightXTranslationPID());

    //SmartDashboard.putNumber("Left Rotation Offset", leftRotationOffset);
    SmartDashboard.putNumber("leftXTranslationOffset", leftXTranslationOffset);
    SmartDashboard.putNumber("leftZTranslationOffset", leftZTranslationOffset);
    SmartDashboard.putNumber("leftXTranslationOffsetToPlacer", leftXTranslationOffsetToPlacer);
    //SmartDashboard.putNumber("left rotation pid output", getLeftRotationPID());
    SmartDashboard.putNumber("left translation pid output", getLeftXTranslationPID());
    
  }
    
  public double getRightRotationPID() {
    return -rotationController.calculate(rightRotationOffset, 0);
  }

  public double getRightXTranslationPID() {
    return -translationController.calculate(rightXTranslationOffsetToPlacer, 0);
  }

  public double getLeftRotationPID() {
    return rotationController.calculate(leftRotationOffset, 0);
  }

  public double getLeftXTranslationPID() {
    return -translationController.calculate(leftXTranslationOffsetToPlacer, 0);
  }

  public boolean isRobotAlignedToLeftReef() {
    if (rightXTranslationOffsetToPlacer <= 0.05) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isRobotAlignedToRightReef() {
    if (rightXTranslationOffsetToPlacer <= 0.05) {
      return true;
    } else {
      return false;
    }
  }

}
