// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PlacerConstants;
import frc.utils.Kraken;

public class Placer extends SubsystemBase {

  Kraken frontMotor, rearMotor;
  DigitalInput placerBeamBreak;

  /** Creates a new Placer. */
  public Placer() {

    // instantiating hardware
    frontMotor = new Kraken(PlacerConstants.frontMotorID);
    rearMotor = new Kraken(PlacerConstants.backMotorID);
    placerBeamBreak = new DigitalInput(PlacerConstants.placerBeamBreakID);

    // configuring motors
    frontMotor.setBrakeMode();
    rearMotor.setBrakeMode();
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
    frontMotor.setMotorSpeed(speed);
  }

  /** Sets the speed of the rear motor. <p> 
   *  Only used for moving forwards. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setRearMotorPlus(double speed) {
    rearMotor.setMotorSpeed(speed);
  }

  /** Sets the speed of the front motor. <p> 
   *  Only used for moving backwards. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setFrontMotorMinus(double speed) {
    frontMotor.setMotorSpeed(-speed);
  }

  /** Sets the speed of the rear motor. <p> 
   *  Only used for moving backwards. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setRearMotorMinus(double speed) {
    rearMotor.setMotorSpeed(-speed);
  }

  /** Gets the state of the beam break sensor. <p>
   *  This sensor is used in the scoring pipeline to determine when the coral is prepared
   *  to be scored and the elevator is safe to move. <p>
   *  Returns false when beam is obstructed and true when beam is unobstructed.
   */
  public boolean getBeamBreakState() { // change to isCoralInPlacer
    return placerBeamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Placer Beam Break State", getBeamBreakState());
    // adds beam break state to smart dashboard
  }

}
