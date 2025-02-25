// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotCoralState extends SubsystemBase {
  /** Creates a new RobotCoralState. */
  Placer placer = new Placer();
  HopperSubsystem hopperSubsystem = new HopperSubsystem();
  boolean robotContainsCoral = true;
  int coralState = 0;
  public RobotCoralState() {}

  public boolean robotContainsCoral(){
    return robotContainsCoral;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(hopperSubsystem.isCoralCollected()&&coralState==0){
      coralState=1;
    }
    if(!placer.getBeamBreakState()){
      coralState=2;
    }
    if(coralState==2&&placer.getBeamBreakState()){
      coralState=0;
      robotContainsCoral = true;
    }
  }
}
