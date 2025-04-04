// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeMechConstants;
import frc.utils.Kraken;

public class AlgaeMech extends SubsystemBase {

  Kraken collectorMotor; //spin both ways
  Kraken pivotMotor; // deploy and undeploy
  
  /** Creates a new AlgaeForklift. */
  public AlgaeMech() {
    collectorMotor = new Kraken(AlgaeMechConstants.collectorMotorID);
    pivotMotor = new Kraken(AlgaeMechConstants.pivotMotorID);
  }

  public void pivotAlgaeMech(double speed) {
    pivotMotor.setMotorSpeed(speed);
  }

  public void manipulateAlgae(double speed) {
    collectorMotor.setMotorSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
