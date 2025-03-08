// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.HopperConstants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private DigitalInput hopperBeamBreak;
  private Debouncer hopperDebouncer;
  private boolean CoralCollected;

  /** Creates a new BeamBreak. */
  public Hopper() {
    CoralCollected = false;
    hopperBeamBreak = new DigitalInput(HopperConstants.hopperBeamBreakID);
    hopperDebouncer = new Debouncer(0, Debouncer.DebounceType.kBoth);
    // Initializes smartdashboard as false
    SmartDashboard.putBoolean("Hopper Beam Break State", CoralCollected);
  }

  public boolean isCoralCollected() {
    return CoralCollected;
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Checks beam break status
    boolean hopperStatus = hopperDebouncer.calculate(hopperBeamBreak.get());

    // Updates smartdashboard status only when beam break value changes
    if(hopperStatus != CoralCollected) {
      SmartDashboard.putBoolean("Hopper Beam Break State", isCoralCollected());
    }

    // Updates CoralCollected value
    CoralCollected = hopperStatus;
  }

  /**
   * Has game piece been collected
   * @return true, false
   */

  
}