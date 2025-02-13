// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PlacerConstants;

public class Placer extends SubsystemBase {

  TalonFX frontMotor;
  TalonFX rearMotor;
  DigitalInput beamBreak;

  /** Creates a new Placer. */
  public Placer() {

    // instantiating hardware
    frontMotor = new TalonFX(PlacerConstants.placerFrontMotorID);
    rearMotor = new TalonFX(PlacerConstants.placerRearMotorID);
    beamBreak = new DigitalInput(PlacerConstants.placerBeamBreakID);

    // configuring motors
    frontMotor.setNeutralMode(NeutralModeValue.Brake);
    rearMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Stops the rear motor. */
  public void stopRearMotor() {
    rearMotor.stopMotor();
  }
  
  /** Stops the front motor. */
  public void stopFrontMotor() {
    frontMotor.stopMotor();
  }

  /** Stops both motors. */
  public void stopBothMotors() {
    frontMotor.stopMotor();
    rearMotor.stopMotor();
  }

  /** Sets the speed of the front motor. <p> 
   *  Only used for moving forwards. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setFrontMotorPlus(double speed) {
    frontMotor.set(speed);
  }

  /** Sets the speed of the rear motor. <p> 
   *  Only used for moving forwards. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setRearMotorPlus(double speed) {
    rearMotor.set(speed);
  }

  /** Sets the speed of the front motor. <p> 
   *  Only used for moving backwards. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setFrontMotorMinus(double speed) {
    frontMotor.set(-speed);
  }

  /** Sets the speed of the rear motor. <p> 
   *  Only used for moving backwards. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setRearMotorMinus(double speed) {
    rearMotor.set(-speed);
  }

  /** Gets the state of the beam break sensor. <p>
   *  This sensor is used in the scoring pipeline to determine when the coral is prepared
   *  to be scored and the elevator is safe to move.
   */
  public boolean getBeamBreakState() {
    return beamBreak.get();
  }
}
