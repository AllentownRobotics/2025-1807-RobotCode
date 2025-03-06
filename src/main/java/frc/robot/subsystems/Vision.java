// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
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

  PIDController rotationController;
  PIDController translationController;

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
    rightRotationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[4];
    rightXTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[0];
    rightZTranslationOffset = frontLimelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6])[2];

    rightXTranslationOffsetToPlacer = rightXTranslationOffset + VisionConstants.placerOffsetToRobotCenter;

    SmartDashboard.putNumber("Right Rotation Offset", rightRotationOffset);
    SmartDashboard.putNumber("rightXTranslationOffset", rightXTranslationOffset);
    SmartDashboard.putNumber("rightZTranslationOffset", rightZTranslationOffset);
    SmartDashboard.putNumber("rightXTranslationOffsetToPlacer", rightXTranslationOffsetToPlacer);
    SmartDashboard.putNumber("right rotation pid output", getRightRotationPID());
    SmartDashboard.putNumber("right translation pid output", getRightXTranslationPID());

  }

  public double getRightRotationPID() {
    return -rotationController.calculate(rightRotationOffset, 0);
  }

  public double getRightXTranslationPID() {
    return -translationController.calculate(rightXTranslationOffsetToPlacer, 0);
  }
}
