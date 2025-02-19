// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {

  Orchestra music;
  TalonFX kraken;

  /** Creates a new Music. */
  public Music() {

    music = new Orchestra();
    kraken = new TalonFX(0);
    music.loadMusic("panda.chrp");
    music.addInstrument(kraken);
  }

  public void Play() {
    music.play();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
