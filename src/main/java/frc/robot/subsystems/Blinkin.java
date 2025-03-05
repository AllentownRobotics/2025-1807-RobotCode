// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;


public class Blinkin extends SubsystemBase {
  /** Creates a new blinkin. */
   Spark blinkin;

    EnumMap<BlinkinConstants.LEDPattern, Double> patternMap = new EnumMap<>(BlinkinConstants.LEDPattern.class);
    public Blinkin() {
      blinkin = new Spark(BlinkinConstants.blinkinID);

    
    patternMap.put(BlinkinConstants.LEDPattern.OFF, BlinkinConstants.off);
    patternMap.put(BlinkinConstants.LEDPattern.IDLE, BlinkinConstants.idle);
    patternMap.put(BlinkinConstants.LEDPattern.ALERT_HUMAN_PLAYER, BlinkinConstants.defaultColor);//color tbd
    patternMap.put(BlinkinConstants.LEDPattern.ALIGNED_WITH_REEF, BlinkinConstants.alignedWithReef);
    patternMap.put(BlinkinConstants.LEDPattern.CELEBRATE, BlinkinConstants.defaultColor);//color tbd
    patternMap.put(BlinkinConstants.LEDPattern.CORAL_COLLECTED, BlinkinConstants.defaultColor);//color tbd
    patternMap.put(BlinkinConstants.LEDPattern.CORAL_INDEXED, BlinkinConstants.defaultColor);//color tbd
    patternMap.put(BlinkinConstants.LEDPattern.CORAL_PLACING, BlinkinConstants.defaultColor);//color tbd
    patternMap.put(BlinkinConstants.LEDPattern.CORAL_PLACED, BlinkinConstants.defaultColor);///color tbd
    patternMap.put(BlinkinConstants.LEDPattern.ELEVATOR_AT_DESIRED_POSITION, BlinkinConstants.defaultColor);//color tbd
    patternMap.put(BlinkinConstants.LEDPattern.CLIMBING, BlinkinConstants.defaultColor);//color tbd
    patternMap.put(BlinkinConstants.LEDPattern.CLIMB_COMPLETE, BlinkinConstants.climbUndecided);
    patternMap.put(BlinkinConstants.LEDPattern.CLIMB_COMPLETE_RED, BlinkinConstants.climbLava);
    patternMap.put(BlinkinConstants.LEDPattern.CLIMB_COMPLETE_BLUE, BlinkinConstants.climbOcean);

    }

    //create a general statement for making a color 
    public void setPattern(double pattern){
      blinkin.set(pattern);
    }

    //set LED color pattern based on robot state
    public void setPattern(BlinkinConstants.LEDPattern statePattern){
      double pattern;
      
      //get color patterns from map
      pattern = patternMap.get(statePattern);

      //if the climb complete pattern was requested, customise it by alliance color
      if (statePattern == BlinkinConstants.LEDPattern.CLIMB_COMPLETE){
        pattern = patternMap.get(statePattern);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
          if(alliance.get()== Alliance.Red){
           pattern = patternMap.get(BlinkinConstants.LEDPattern.CLIMB_COMPLETE_RED);
          }
          if(alliance.get()== Alliance.Blue){
            pattern = patternMap.get(BlinkinConstants.LEDPattern.CLIMB_COMPLETE_BLUE);
          }
        }
      }

      //set the pattern
      blinkin.set(pattern);
    }

    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
