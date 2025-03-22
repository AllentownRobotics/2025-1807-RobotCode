// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  Pose2d frontPose;
  Pose2d hopperPose;

  boolean frontTv;
  boolean hopperTv;

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

    frontTv = frontLimelightTable.getEntry("tv").getInteger(0) > 0;
    hopperTv = hopperLimelightTable.getEntry("tv").getInteger(0) > 0;

    // right side reef targeting
    rightRotationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    rightXTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
    rightZTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

    rightXTranslationOffsetToPlacer = rightXTranslationOffset + VisionConstants.rightSideTargetingPlacerOffsetToRobotCenter;

    frontPose = new Pose2d(new Translation2d(rightXTranslationOffset, rightZTranslationOffset),
                                  Rotation2d.fromDegrees(rightRotationOffset));

    //left side reef targeting
    leftRotationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    leftXTranslationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
    leftZTranslationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

    leftXTranslationOffsetToPlacer = leftXTranslationOffset + VisionConstants.leftSideTargetingPlacerOffsetToRobotCenter;

    hopperPose = new Pose2d(new Translation2d(leftXTranslationOffset, leftZTranslationOffset),
                                    Rotation2d.fromDegrees(leftRotationOffset));

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

  public boolean isTargetInFront() {
    return frontTv || hopperTv;
  }

  public Optional<Pose2d> frontPoseTargetSpace() {
    if(!isTargetInFront()) {
      return Optional.empty();
    }

    int frontExists = frontTv ? 1 : 0;
    int hopperExists = hopperTv ? 1 : 0;

    Pose2d newFrontPose = frontPose.times(frontExists);
    Pose2d newHopperPose = hopperPose.times(hopperExists);

    Translation2d translationPoseFront = newFrontPose.getTranslation();
    Rotation2d rotationPoseFront = newFrontPose.getRotation();

    Translation2d translationPoseHopper = newHopperPose.getTranslation();
    Rotation2d rotationPoseHopper = newHopperPose.getRotation();

    return Optional.of(new Pose2d(translationPoseFront.plus(translationPoseHopper).div(frontExists + hopperExists),
                        rotationPoseFront.plus(rotationPoseHopper).div(frontExists + hopperExists)));
  }


  

  public double getRightRotationPID() {
    return -rotationController.calculate(rightRotationOffset, 0);
  }

  public double getRightXTranslationPID() {
    return translationController.calculate(rightXTranslationOffsetToPlacer, 0);
  }

  public double getLeftRotationPID() {
    return -rotationController.calculate(leftRotationOffset, 0);
  }

  public double getLeftXTranslationPID() {
    return translationController.calculate(leftXTranslationOffsetToPlacer, 0); // old: 13.526
    //center of robot to placer
  }

  public boolean isRobotAlignedToLeftReef() {
    if ((Math.abs(leftXTranslationOffsetToPlacer) <= 0.05) 
    //&& (Math.abs(leftRotationOffset) <= 2.00)
    ) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isRobotAlignedToRightReef() {
    if ((Math.abs(rightXTranslationOffsetToPlacer) <= 0.05) && (Math.abs(rightRotationOffset) <= 2.00)) {
      return true;
    } else {
      return false;
    }
  }

}
