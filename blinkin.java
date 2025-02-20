// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class blinkin extends SubsystemBase {
  /** Creates a new blinkin. */
  Spark blinkin; 

  public blinkin() {
    blinkin = new Spark(0);

  }
    public void color(double speed){
      blinkin.set(speed);
    }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
