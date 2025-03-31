// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

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
  double rightYTranslationOffset;
  double rightZTranslationOffset;

  double leftRotationOffset;
  double leftXTranslationOffset;
  double leftYTranslationOffset;
  double leftZTranslationOffset;

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
    
    }


    public boolean isTargetInFront() {
      return frontTv || hopperTv;
    }

    public Optional<Pose2d> frontPoseTargetSpace() {
      if(!isTargetInFront()) {
        return Optional.empty();
      }

      // 1 means that it sees an april tag, 0 means it does not
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

    public boolean isNearPoseFrontTargetSpace(Pose2d pose) {
      
      if(!isTargetInFront()) {
        return false;
      }

      Pose2d visionPose = frontPoseTargetSpace().get();
      double distance = pose.getTranslation().getDistance(visionPose.getTranslation());
      
      double angle = Math.abs(pose.getRotation().getDegrees() - visionPose.getRotation().getDegrees());

      return distance <= VisionConstants.distanceDeadzone && angle <= VisionConstants.angleDeadzone;
      
    }

      @Override
      public void periodic() {
        // This method will be called once per scheduler run

        frontTv = frontLimelightTable.getEntry("tv").getInteger(0) > 0;
        hopperTv = hopperLimelightTable.getEntry("tv").getInteger(0) > 0;

        // right side reef values
        rightRotationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
        rightXTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
        rightYTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[1];
        rightZTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

        frontPose = new Pose2d(new Translation2d(rightXTranslationOffset, rightZTranslationOffset),
                                      Rotation2d.fromDegrees(rightRotationOffset));

        //left side reef values
        leftRotationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
        leftXTranslationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
        leftYTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[1];
        leftZTranslationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

        hopperPose = new Pose2d(new Translation2d(leftXTranslationOffset, leftZTranslationOffset),
                                        Rotation2d.fromDegrees(leftRotationOffset));

        // SmartDashboard.putNumber("right rotation offset", rightRotationOffset);
        // SmartDashboard.putNumber("rightXTranslationOffset", rightXTranslationOffset);
        // SmartDashboard.putNumber("rightYTranslationOffset", rightYTranslationOffset);
        // SmartDashboard.putNumber("rightZTranslationOffset", rightZTranslationOffset);

        // SmartDashboard.putNumber("left rotation offset", leftRotationOffset);
        // SmartDashboard.putNumber("leftXTranslationOffset", leftXTranslationOffset);
        // SmartDashboard.putNumber("leftYTranslationOffset", leftYTranslationOffset);
        // SmartDashboard.putNumber("leftZTranslationOffset", leftZTranslationOffset);

      }
    }