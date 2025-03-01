// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An abstract motor object that contains all necessary configurations for the motors. */
public class Kraken extends SubsystemBase {

  // https://pro.docs.ctr-electronics.com/en/2023-pro/docs/api-reference/api-usage/configuration.html
  // https://github.com/PeddieRobotics/2024Itsumade/blob/dev/2024-I/src/main/java/frc/robot/utils/Kraken.java
  // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/index.html 

  TalonFX kraken;
  TalonFXConfiguration krakenConfiguration;
  CANcoderConfiguration encoderConfiguration;
  int krakenID;
  Slot0Configs slot0Configs;
  Orchestra music;

  /** Creates a new Kraken. */
  public Kraken(int krakenID) {
    // Default motor configs are here. Additional motor configs can be applied with the methods listed below.
    this.kraken = new TalonFX(krakenID);
    this.krakenID = krakenID;
    krakenConfiguration = new TalonFXConfiguration();
    kraken.getConfigurator().setPosition(0);

    music = new Orchestra();
    music.loadMusic("panda.chrp");
    music.addInstrument(kraken);
  }

  /** Factory resets the motor. */
  public void restoreFactoryDefaults() {
    kraken.getConfigurator().apply(new TalonFXConfiguration());
  }

  /** Resets built-in encoder position. */
  public void resetEncoder() {
    kraken.getConfigurator().setPosition(0);
  }

  /** Sets an encoder position depending on user generated values. */
  public void setDesiredEncoderPosition(double position) {
    kraken.getConfigurator().setPosition(position);
  }

  /** Sets motor speed to zero. */
  public void stopMotor() {
    kraken.set(0);
  }

  /** Sets default motor config inverted. */
  public void setInverted() {  
    kraken.getConfigurator().apply(krakenConfiguration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
  }

  /** Sets default motor config not inverted. */
  public void setNotInverted() {  
    kraken.getConfigurator().apply(krakenConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
  }

  /** Puts disabled motor in brake mode. */
  public void setBrakeMode() {
    kraken.getConfigurator().apply(krakenConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Brake));
  }

  /** Puts disabled motor in coast mode. */
  public void setCoastMode() {
    kraken.getConfigurator().apply(krakenConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Coast));
  }

  /** Sets the current limits (in amps) on the motor. Takes in values from -1.0 to 1.0. */
  public void setMotorCurrentLimits(double amps) {
    kraken.getConfigurator().apply(krakenConfiguration.CurrentLimits.withSupplyCurrentLimit(amps));
  }

  /** Sets the motor's speed. Takes in values from -1.0 to 1.0. */
  public void setMotorSpeed(double speed) {
    kraken.set(speed);
  }

  /** Allows a motor to follow another motor. Setting the inverted boolean to true allows motor to turn opposite the leader. */
  public void follow(double leaderCANID, boolean inverted) {
    kraken.setControl(new Follower(krakenID, inverted));
  }

  /** Connects a CANCoder to the specified motor. */
  public void addEncoder(CoreCANcoder encoder) {
    kraken.getConfigurator().apply(krakenConfiguration.Feedback.withRemoteCANcoder(encoder));
  }

  /** Returns the device temperature. */
  public void getMotorTemperature() {
    kraken.getDeviceTemp();
  }

  /** Returns the position of the encoder in rotations. */
  public void getEncoderPosition(CoreCANcoder encoder) {
    encoder.getPosition();
  }

  /** Returns the position of the motor. */
  public double getPosition() {
    return kraken.getPosition().getValueAsDouble();
  }

 /** Returns the velocity of the motor. */
  public double getVelocity() {
    return kraken.get();
  }
  
  public void setMotionMagicParameters(double maxVelocity, double maxAccel, double maxJerk) {
    krakenConfiguration.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    krakenConfiguration.MotionMagic.MotionMagicAcceleration = maxAccel;
    krakenConfiguration.MotionMagic.MotionMagicJerk = maxJerk;

    kraken.getConfigurator().apply(krakenConfiguration);
  }

  //** Allows preset PID values to be passed to the motor. */
  public void setPIDValues(double kP, double kI, double kD, double kS, double kV, double kA) {
    krakenConfiguration.Slot0.kP = kP;
    krakenConfiguration.Slot0.kI = kI;
    krakenConfiguration.Slot0.kD = kD;
    krakenConfiguration.Slot0.kS = kS;
    krakenConfiguration.Slot0.kV = kV;
    krakenConfiguration.Slot0.kA = kA;

    kraken.getConfigurator().apply(krakenConfiguration);
  }

  /** PID soft limits are built in limit switches for the motor. */
  public void setSoftLimits(double reverseLimit, double forwardLimit) {
    kraken.getConfigurator().apply(krakenConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(reverseLimit));
    kraken.getConfigurator().apply(krakenConfiguration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(forwardLimit));
  }

  /** Plays loaded music soundtrack. */
  public void playMusic() {
    music.play();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
