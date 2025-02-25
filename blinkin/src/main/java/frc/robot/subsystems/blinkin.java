// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class blinkin extends SubsystemBase {
  /** Creates a new blinkin. */
  static Spark blinkin; 
  
    public blinkin() {
      blinkin = new Spark(0);
  
    } 
      public static void color(double color){
        blinkin.set(color);
    }

   /* private static boolean flashing = false;


        public void setFlashColor(int white){
          this. = new Color(white);
        }
    
        public static void startFlash(){
          flashing = true;

    }

    public static void stopFlash(){
      flashing = false;
    }*/

    


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
