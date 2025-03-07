// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.Kraken;
import java.util.function.BooleanSupplier;

public class Elevator extends SubsystemBase {

  private Kraken leftMotor, rightMotor;
  private CANcoder elevatorEncoder;
  private DigitalInput lowerLimitSwitch, upperLimitSwitch;
  private double desiredSetpoint; // desired setpoint of the encoder

  private SysIdRoutine appliedRoutine;

  private ElevatorFeedforward feedforward;

  //https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
  private final SysIdRoutine elevatorSysIDRoutine = new SysIdRoutine(
    new Config(
      Volts.of(.5).per(Second),
      Volts.of(1),
      null,
      (state) -> SignalLogger.writeString("elevator sysID State", state.toString())),
    new Mechanism(
      (volts) -> leftMotor.setVolts(volts),
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
    
    leftMotor.setInverted();
    
    rightMotor.follow(ElevatorConstants.leftMotorID, false);
    
    leftMotor.addEncoder(elevatorEncoder);
    
    leftMotor.setRotorToSensorRatio(ElevatorConstants.elevatorGearing);
    leftMotor.setSensorToMechanismRatio(ElevatorConstants.elevatorEncoderToMechanismRatio);
    
    
    leftMotor.setPIDValues(ElevatorConstants.ELEVATOR_P, ElevatorConstants.ELEVATOR_I,
    ElevatorConstants.ELEVATOR_D, ElevatorConstants.ELEVATOR_SFF,
    ElevatorConstants.ELEVATOR_VFF, ElevatorConstants.ELEVATOR_AFF, ElevatorConstants.ELEVATOR_GFF);
    
    leftMotor.setBrakeMode();
    rightMotor.setBrakeMode();
    
    leftMotor.setMotorCurrentLimits(40);
    leftMotor.setSoftLimits(ElevatorConstants.softLimitMinPosition, // prevent us from overdriving the motor
    ElevatorConstants.softLimitMaxPosition);
    
    desiredSetpoint = ElevatorConstants.homePosition;
    elevatorEncoder.setPosition(0);
    leftMotor.setDesiredEncoderPosition(desiredSetpoint);
    
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

  public void setPercent(double percent) {
    leftMotor.setMotorSpeed(percent);
  }

  /** Stop elevator (emergency feature). */
  public void stopElevatorVolts() {
    leftMotor.setVolts(Voltage.ofBaseUnits(0, Volts));
  }

  /** Sets elevator position based off setpoint value. */
  public void setElevatorPosition(double setpoint) {
    desiredSetpoint = setpoint;
    leftMotor.setDesiredEncoderPosition(desiredSetpoint);
  }

  /** Adjust the position of the elevator by a set increment. */
  public void adjustPositionIncrementally(double increment) {
    desiredSetpoint += increment;
    leftMotor.setDesiredEncoderPosition(desiredSetpoint);
  }

  /** Scales the elevator encoder position by a factor of 2(pi)(r), converting rotations to inches. */
  public double getElevatorPositionInInches() {
    return leftMotor.getPosition();
  }

  public boolean isLowerLimitReached() {
    return lowerLimitSwitch.get();
  }

  public boolean isUpperLimitReached() {
    return upperLimitSwitch.get();
  }

  public BooleanSupplier isAtPosition(double targetPosition) {

    // if ( Units.epsilonEquals(getElevatorPositionInInches(), position, Constants.ElevatorConstants.positionTolerance )) {
    double currentPosition = getElevatorPositionInInches();
    if ( (targetPosition - Constants.ElevatorConstants.positionTolerance >= currentPosition ) && 
         (targetPosition + Constants.ElevatorConstants.positionTolerance <= currentPosition)) {
      return () -> true;
    }
    
    return () -> false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    leftMotor.getMotorTemperature();
    rightMotor.getMotorTemperature();

    /*if (isLowerLimitReached() == true) {
      leftMotor.setMotorSpeed(0); //replace these numbers to spin the motors away from the limit switch
    }

    if (isUpperLimitReached() == true) {
      leftMotor.setMotorSpeed(-.1); //replace these numbers to spin the motors away from the limit switch
    }*/

    // change state only when state changes
    SmartDashboard.putNumber("elevator encoder desired position", desiredSetpoint);
    SmartDashboard.putBoolean("elevator min limit", !isLowerLimitReached());
    SmartDashboard.putBoolean("elevator max limit", !isUpperLimitReached());
    SmartDashboard.putNumber("elevator height", getElevatorPositionInInches());
    SmartDashboard.putNumber("elevator encoder heihgt", elevatorEncoder.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("right elevator draw", rightMotor.getSupplyCurrent());
    SmartDashboard.putNumber("left elevator draw", leftMotor.getSupplyCurrent());
    
  }

}
