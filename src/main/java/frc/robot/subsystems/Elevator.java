// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private Kraken leftMotor, rightMotor;
  private CANcoder elevatorEncoder;
  private DigitalInput lowerLimitSwitch, upperLimitSwitch;
  private double desiredSetpoint; // desired setpoint of the encoder

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new Kraken(ElevatorConstants.leftMotorID);
    rightMotor = new Kraken(ElevatorConstants.rightMotorID);
    elevatorEncoder = new CANcoder(ElevatorConstants.elevatorCANCoderID);

    lowerLimitSwitch = new DigitalInput(ElevatorConstants.lowerLimitSwitchPort);
    upperLimitSwitch = new DigitalInput(ElevatorConstants.upperLimitSwitchPort);

    rightMotor.follow(ElevatorConstants.leftMotorID, false);
    leftMotor.restoreFactoryDefaults();

    leftMotor.addEncoder(elevatorEncoder);

    leftMotor.setPIDValues(ElevatorConstants.ELEVATOR_P, ElevatorConstants.ELEVATOR_I,
                              ElevatorConstants.ELEVATOR_D, ElevatorConstants.ELEVATOR_SFF,
                              ElevatorConstants.ELEVATOR_VFF, ElevatorConstants.ELEVATOR_AFF);

    leftMotor.setBrakeMode();

    leftMotor.setMotorCurrentLimits(120);
    leftMotor.setSoftLimits(ElevatorConstants.homePosition, ElevatorConstants.L4Position); // prevent us from overdriving the motor
    
    desiredSetpoint = ElevatorConstants.homePosition;
    elevatorEncoder.setPosition(desiredSetpoint);

  }

  /** Sets elevator position based off setpoint value. */
  public void setElevatorPosition(double setpoint) {
    desiredSetpoint = setpoint;
  }

  /** Adjust the position of the elevator by a set increment. */
  public void adjustPositionIncrementally(double increment) {
    desiredSetpoint += increment;
  }

  public void limitSwitchStatus() {
    lowerLimitSwitch.get();
    upperLimitSwitch.get();
  }

  private final SysIdRoutine elevatorSysIDRoutine = new SysIdRoutine(
    new Config(
      null,
      Voltage.ofBaseUnits(4, Volts),
      null),
    new Mechanism(
      null,
      null,
      null));

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Shuffleboard.getTab("Elevator").add("encoder position", 0);
  }
}
