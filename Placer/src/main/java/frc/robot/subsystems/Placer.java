// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PlacerConstants;

public class Placer extends SubsystemBase {

  private Kraken frontMotor;
  private Kraken rearMotor;
  private DigitalInput beamBreak;
  private boolean previousPlacerCoralState;

  /** Creates a new Placer. */
  public Placer() {

    // instantiating hardware
    frontMotor = new Kraken(PlacerConstants.placerFrontMotorID);
    rearMotor = new Kraken(PlacerConstants.placerRearMotorID);
    beamBreak = new DigitalInput(PlacerConstants.placerBeamBreakID);

    // configuring motors
    frontMotor.setBrakeMode();
    rearMotor.setBrakeMode();

    // sets up coral states to update on first scheduler tick
    previousPlacerCoralState = isCoralInPlacer();

    // sends inital coral state into smart dash
    SmartDashboard.putBoolean("Beam Break State", previousPlacerCoralState);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // adds beam break state to smart dashboard
    // only if it changes states to reduce smartdash load
    boolean placerCoralState = isCoralInPlacer();
    if (placerCoralState != previousPlacerCoralState) {

      SmartDashboard.putBoolean("Beam Break State", placerCoralState);
      previousPlacerCoralState = placerCoralState;
    }
    
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
  public void setFrontMotorForwards(double speed) {
    frontMotor.setMotorSpeed(speed);
  }

  /** Sets the speed of the rear motor. <p> 
   *  Only used for moving forwards. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setRearMotorForwards(double speed) {
    rearMotor.setMotorSpeed(speed);
  }

  /** Sets the speed of the front motor. <p> 
   *  Only used for moving in reverse. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setFrontMotorReverse(double speed) {
    frontMotor.setMotorSpeed(-speed);
  }

  /** Sets the speed of the rear motor. <p> 
   *  Only used for moving in reverse. <p>
   *  Use positive numbers for the speed variable only.
   */
  public void setRearMotorReverse(double speed) {
    rearMotor.setMotorSpeed(-speed);
  }

  /** Gets the state of the beam break sensor. <p>
   *  This sensor is used in the scoring pipeline to determine when the coral is prepared
   *  to be scored and the elevator is safe to move. <p>
   *  Returns false when beam is obstructed and true when beam is unobstructed
   */
  public boolean isCoralInPlacer() {
    return beamBreak.get();
  }
}
