// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;

public class Kraken extends SubsystemBase {

  // https://pro.docs.ctr-electronics.com/en/2023-pro/docs/api-reference/api-usage/configuration.html
  // https://github.com/PeddieRobotics/2024Itsumade/blob/dev/2024-I/src/main/java/frc/robot/utils/Kraken.java

  TalonFX kraken;
  TalonFXConfiguration krakenConfiguration;
  int krakenID;
  Slot0Configs slot0Configs;
  Orchestra music;

  /** Creates a new Kraken. */
  public Kraken(int krakenID) {
    // default motor configs are here. Additional motor configs can be applied with the methods listed below.
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

  /** Resets encoder position. */
  public void resetEncoder() {
    kraken.getConfigurator().setPosition(0);
  }

  /** Sets an encoder position depending on user generated values. */
  public void setDesiredEncoderPosition(double position) {
    kraken.getConfigurator().setPosition(position);
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

  /** Allows a motor to follow another motor. */
  public void setFollowerMotor(double leaderCANID, boolean inverted) {
    kraken.setControl(new Follower(krakenID, inverted));
  }

  // use for srx mag encoders
  public double getEncoderPosition() {
    return 0;
  }

  /** Returns the position of the motor. */
  public double getPosition() {
    return kraken.getPosition().getValueAsDouble();
  }

 /** Returns the velocity of the motor. */
  public double getVelocity() {
    return kraken.get();
  }

  // motion magic takes a lot of guesswork out of PID loops.
  // documentation: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html

  public void setMotionMagicParameters(double maxVelocity, double maxAccel, double maxJerk) {
    krakenConfiguration.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    krakenConfiguration.MotionMagic.MotionMagicAcceleration = maxAccel;
    krakenConfiguration.MotionMagic.MotionMagicJerk = maxJerk;

    kraken.getConfigurator().apply(krakenConfiguration);
  }

  public void setPIDValues(double kP, double kI, double kD, double kS, double kV, double kA) {
    krakenConfiguration.Slot0.kP = kP;
    krakenConfiguration.Slot0.kI = kI;
    krakenConfiguration.Slot0.kD = kD;
    krakenConfiguration.Slot0.kS = kS;
    krakenConfiguration.Slot0.kV = kV;
    krakenConfiguration.Slot0.kA = kA;

    kraken.getConfigurator().apply(krakenConfiguration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
