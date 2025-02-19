// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  Kraken elevatorMotor;
  TalonSRX magEncoder;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new Kraken(ElevatorConstants.elevatorMotorID);
    magEncoder = new TalonSRX(0);

    elevatorMotor.restoreFactoryDefaults();
    
    elevatorMotor.setPIDValues(ElevatorConstants.ELEVATOR_P, ElevatorConstants.ELEVATOR_I,
                              ElevatorConstants.ELEVATOR_D, ElevatorConstants.ELEVATOR_SFF,
                              ElevatorConstants.ELEVATOR_VFF, ElevatorConstants.ELEVATOR_AFF);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
