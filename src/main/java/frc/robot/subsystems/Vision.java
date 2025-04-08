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
  NetworkTable backLimelightTable;

  double rightRotationOffset;
  double rightXTranslationOffset;
  double rightYTranslationOffset;
  double rightZTranslationOffset;

  double leftRotationOffset;
  double leftXTranslationOffset;
  double leftYTranslationOffset;
  double leftZTranslationOffset;

  double backRotationOffset;
  double backXTranslationOffset;
  double backYTranslationOffset;
  double backZTranslationOffset;

  Pose2d frontPose;
  Pose2d hopperPose;
  Pose2d backPose;

  boolean frontTv;
  boolean hopperTv;
  boolean backTv;
  
  /** Creates a new Vision. */
  public Vision() {
    
    frontLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
    hopperLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-hopper");
    backLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-back");
    
    frontLimelightTable.getEntry("priorityid").setNumber(-1);
    hopperLimelightTable.getEntry("priorityid").setNumber(-1);
    backLimelightTable.getEntry("priorityid").setNumber(-1);
    }
    

    public boolean isTargetInFront() {
      return frontTv || hopperTv;
    }

    public boolean isTargetInBack() {
      return backTv;
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

    public boolean isNearPoseFrontTargetSpace(Pose2d pose, double ydeadzone) {
      
      if(!isTargetInFront()) {
        return false;
      }

      Pose2d frontVisionPose = frontPoseTargetSpace().get();

      double xdistance = Math.abs(pose.getTranslation().getX() - frontVisionPose.getTranslation().getX());
      double ydistance = Math.abs(pose.getTranslation().getY() - frontVisionPose.getTranslation().getY());
      double angle = Math.abs(pose.getRotation().getDegrees() - frontVisionPose.getRotation().getDegrees());

      boolean frontXInRange = (xdistance <= VisionConstants.xDistanceDeadzone);
      boolean frontYInRange = (ydistance <= ydeadzone);
      boolean frontRotInRange = (angle <= VisionConstants.angleDeadzone);

      boolean isFrontInRange = frontXInRange && frontYInRange && frontRotInRange;

      SmartDashboard.putNumber("x distance from aligned", xdistance);
      SmartDashboard.putNumber("y distance from aligned", ydistance);
      SmartDashboard.putNumber("angular distance from aligned", angle);

      SmartDashboard.putBoolean("x in range", frontXInRange);
      SmartDashboard.putBoolean("y in range", frontYInRange);
      SmartDashboard.putBoolean("rot in range", frontRotInRange);
      SmartDashboard.putBoolean("target in range", isFrontInRange);

      return isFrontInRange;
    }

    public Optional<Pose2d> backPoseTargetSpace() {
      if(!isTargetInBack()) {
        return Optional.empty();
      }

      int backExists = backTv ? 1 : 0;

      Pose2d newBackPose = backPose.times(backExists);

      Translation2d translationPoseBack = newBackPose.getTranslation();
      Rotation2d rotationPoseBack = newBackPose.getRotation();

      return Optional.of(new Pose2d(translationPoseBack.div(backExists),
                          rotationPoseBack.div(backExists)));
    }

    public boolean isNearPoseBackTargetSpace(Pose2d backPose) {

      if(!isTargetInBack()) {
        return false;
      }

      Pose2d backVisionPose = backPoseTargetSpace().get();

      double xdistance = Math.abs(backPose.getTranslation().getX() - backVisionPose.getTranslation().getX());
      double ydistance = Math.abs(backPose.getTranslation().getY() - backVisionPose.getTranslation().getY());
      double angle = Math.abs(backPose.getRotation().getDegrees() - backVisionPose.getRotation().getDegrees());

      boolean backXInRange = (xdistance <= VisionConstants.xDistanceDeadzone);
      boolean backYInRange = (ydistance <= VisionConstants.yLeftReefDistanceDeadzone);
      boolean backRotInRange = (angle <= VisionConstants.angleDeadzone);

      boolean isBackInRange = backXInRange && backYInRange && backRotInRange;

      SmartDashboard.putNumber("x distance from aligned", xdistance);
      SmartDashboard.putNumber("y distance from aligned", ydistance);
      SmartDashboard.putNumber("angular distance from aligned", angle);

      SmartDashboard.putBoolean("x in range", backXInRange);
      SmartDashboard.putBoolean("y in range", backYInRange);
      SmartDashboard.putBoolean("rot in range", backRotInRange);
      SmartDashboard.putBoolean("target in range", isBackInRange);

      return isBackInRange;

    }

      @Override
      public void periodic() {
        // This method will be called once per scheduler run

        frontTv = frontLimelightTable.getEntry("tv").getInteger(0) > 0;
        hopperTv = hopperLimelightTable.getEntry("tv").getInteger(0) > 0;
        backTv = backLimelightTable.getEntry("tv").getInteger(0) > 0;

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
        leftYTranslationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[1];
        leftZTranslationOffset = hopperLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

        hopperPose = new Pose2d(new Translation2d(leftXTranslationOffset, leftZTranslationOffset),
                                        Rotation2d.fromDegrees(leftRotationOffset));

        //human player station values
        backRotationOffset = backLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
        backXTranslationOffset = backLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
        backYTranslationOffset = backLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[1];
        backZTranslationOffset = backLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

        backPose = new Pose2d(new Translation2d(backXTranslationOffset, backZTranslationOffset),
                                      Rotation2d.fromDegrees(backRotationOffset));

        SmartDashboard.putNumber("right rotation offset", rightRotationOffset);
        SmartDashboard.putNumber("rightXTranslationOffset", rightXTranslationOffset);
        SmartDashboard.putNumber("rightYTranslationOffset", rightYTranslationOffset);
        SmartDashboard.putNumber("rightZTranslationOffset", rightZTranslationOffset);

        SmartDashboard.putNumber("left rotation offset", leftRotationOffset);
        SmartDashboard.putNumber("leftXTranslationOffset", leftXTranslationOffset);
        SmartDashboard.putNumber("leftYTranslationOffset", leftYTranslationOffset);
        SmartDashboard.putNumber("leftZTranslationOffset", leftZTranslationOffset);

      }
    }