// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Kraken extends SubsystemBase {

  TalonFX kraken;
  TalonFXConfiguration krakenConfiguration;
  int krakenID;
  Slot0Configs krakenSlot0Configs;
  Orchestra music;

  /** Creates a new Kraken. */
  public Kraken(int krakenID) {
    this.kraken = new TalonFX(krakenID);
    

    krakenSlot0Configs = new Slot0Configs()
    .withKP(ElevatorConstants.ELEVATOR_P).withKI(ElevatorConstants.ELEVATOR_I).withKD(ElevatorConstants.ELEVATOR_D)
    .withKS(ElevatorConstants.ELEVATOR_SFF).withKV(ElevatorConstants.ELEVATOR_VFF).withKA(ElevatorConstants.ELEVATOR_AFF);
    
    krakenConfiguration = new TalonFXConfiguration().withSlot0(krakenSlot0Configs);

  //add configs and stuff https://pro.docs.ctr-electronics.com/en/2023-pro/docs/api-reference/api-usage/configuration.html
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
