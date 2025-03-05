// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.Kraken;

public class Elevator extends SubsystemBase {

  private Kraken leftMotor, rightMotor;
  private CANcoder elevatorEncoder;
  private DigitalInput lowerLimitSwitch, upperLimitSwitch;
  private double desiredSetpoint; // desired setpoint of the encoder

  private SysIdRoutine appliedRoutine;

  //https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
  private final SysIdRoutine elevatorSysIDRoutine = new SysIdRoutine(
    new Config(
      null,
      Voltage.ofBaseUnits(2, Volts),
      Seconds.of(4)),
    new Mechanism(
      state -> SignalLogger.writeString("elevator sysID State", state.toString()),
      null,
      this)
      );

  /** Creates a new Elevator. */
  public Elevator() {

    appliedRoutine = elevatorSysIDRoutine;
    
    /*if(Robot.isSimulation()) {
      new ElevatorSim(
        DCMotor.getKrakenX60(2),
        ElevatorConstants.elevatorGearing,
        ElevatorConstants.carriageMass,
        ElevatorConstants.elevatorSprocketRadius,
        ElevatorConstants.minHeightMeters,
        ElevatorConstants.maxHeightMeters,
        true,
        ElevatorConstants.startingHeightMeters);*/

    leftMotor = new Kraken(ElevatorConstants.leftMotorID);
    rightMotor = new Kraken(ElevatorConstants.rightMotorID);
    elevatorEncoder = new CANcoder(ElevatorConstants.elevatorCANCoderID);

    lowerLimitSwitch = new DigitalInput(ElevatorConstants.lowerLimitSwitchPort);
    upperLimitSwitch = new DigitalInput(ElevatorConstants.upperLimitSwitchPort);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    rightMotor.follow(ElevatorConstants.leftMotorID, true);
    
    leftMotor.addEncoder(elevatorEncoder);

    leftMotor.setRotorToSensorRatio(ElevatorConstants.elevatorGearing);
    leftMotor.setSensorToMechanismRatio(ElevatorConstants.elevatorSprocketCircumference);

    leftMotor.setPIDValues(ElevatorConstants.ELEVATOR_P, ElevatorConstants.ELEVATOR_I,
                              ElevatorConstants.ELEVATOR_D, ElevatorConstants.ELEVATOR_SFF,
                              ElevatorConstants.ELEVATOR_VFF, ElevatorConstants.ELEVATOR_AFF);

    leftMotor.setBrakeMode();
    leftMotor.setInverted(); //changed from not inverted (may need to be fixed) <(||._.)

    leftMotor.setMotorCurrentLimits(40);
    leftMotor.setSoftLimits(ElevatorConstants.softLimitMinPosition, // prevent us from overdriving the motor
                            ElevatorConstants.softLimitMaxPosition); 
    
    desiredSetpoint = ElevatorConstants.homePosition;
    elevatorEncoder.setPosition(desiredSetpoint);

    SignalLogger.start();
    }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return appliedRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return appliedRoutine.dynamic(direction);
  }

  /** Stop elevator (emergency feature). */
  public void stopElevator() {
    leftMotor.stopMotor();
  }

  /** Sets elevator position based off setpoint value. */
  public void setElevatorPosition(double setpoint) {
    desiredSetpoint = setpoint;
    elevatorEncoder.setPosition(desiredSetpoint);
  }

  /** Adjust the position of the elevator by a set increment. */
  public void adjustPositionIncrementally(double increment) {
    desiredSetpoint += increment;
    elevatorEncoder.setPosition(desiredSetpoint);
  }

  /** Scales the elevator encoder position by a factor of 2(pi)(r), converting rotations to inches. */
  public double getElevatorPositionInInches() {
    return elevatorEncoder.getAbsolutePosition().getValueAsDouble()*ElevatorConstants.elevatorSprocketCircumference;
  }

  public boolean isLowerLimitReached() {
    return lowerLimitSwitch.get();
  }

  public boolean isUpperLimitReached() {
    return upperLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    leftMotor.getMotorTemperature();
    rightMotor.getMotorTemperature();

    if (isLowerLimitReached() == true) {
      leftMotor.setMotorSpeed(.1); //replace these numbers to spin the motors away from the limit switch
    }

    if (isUpperLimitReached() == true) {
      leftMotor.setMotorSpeed(-.1); //replace these numbers to spin the motors away from the limit switch
    }

    // change state only when state changes
    SmartDashboard.getNumber("encoder desired position", desiredSetpoint);
    //Shuffleboard.getTab("Elevator").add("at min height", isLowerLimitReached());
    //Shuffleboard.getTab("Elevator").add("at max height", isUpperLimitReached());
    //Shuffleboard.getTab("Elevator").add("elevator position in inches", getElevatorPositionInInches());
    
  }

}
