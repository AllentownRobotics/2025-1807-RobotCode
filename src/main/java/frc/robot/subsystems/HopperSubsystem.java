// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.HopperConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  private DigitalInput hopperBeamBreak;
  private Debouncer hopper_debouncer;
  private static boolean CoralCollected;
  /** Creates a new BeamBreak. */
  public HopperSubsystem() {
    CoralCollected = false;
    hopperBeamBreak = new DigitalInput(HopperConstants.hopperBeamBreakID);
    hopper_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    // Initializes smartdashboard as false
    SmartDashboard.putBoolean("beam break", CoralCollected);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    // Checks beam break status
    boolean hopperStatus = hopper_debouncer.calculate(hopperBeamBreak.get());

    // Updates smartdashboard status only when beam break value changes
    if(hopperStatus != CoralCollected) {
      SmartDashboard.putBoolean("beam break", isCoralCollected());
    }

    // Updates CoralCollected value
    CoralCollected = hopperStatus;
  }

  /**
   * Has game piece been collected
   * @return true, false
   */
  public boolean isCoralCollected() {
    return CoralCollected;
  }
  
}