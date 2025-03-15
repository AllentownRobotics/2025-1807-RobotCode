// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotCoralState extends SubsystemBase {
  
  Placer placer = new Placer();
  Hopper hopperSubsystem = new Hopper();
  boolean robotContainsCoral = true;
  int coralState = 0;
  /** Creates a new RobotCoralState. */
  public RobotCoralState() {}

  public boolean robotContainsCoral(){
    // returns whether the robot contains coral or not
    return robotContainsCoral;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // sets the boolean robot contains coral by checking if the hoper had the coral, and the robot had no other coral, if the placer beam is broken then it sets the state to 2
    // and if the placer beam is not broken and the placer beam has been broken before that, it seta the robot to not contain coral.
    if(hopperSubsystem.isCoralCollected()&&coralState==0){
      robotContainsCoral = true;
      coralState=1;
    }
    if(!placer.isCoralInPlacer()){
      coralState=2;
    }
    if(coralState==2&&placer.isCoralInPlacer()){
      coralState=0;
      robotContainsCoral = false;
    }
    SmartDashboard.putBoolean("Has Coral", robotContainsCoral);
  }
}
