// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  DigitalInput outerClimbLimitSwitch, innerClimbLimitSwitch, cageContactLimitSwitch;
  Kraken rightClimbMotor;
  Kraken leftClimbMotor;
  TalonFX motor;
  CANcoder climbCANcoder;
  double desiredAngle;

  /** Creates a new Climb. */
  public Climb() {
    motor = new TalonFX(1);
    rightClimbMotor = new Kraken(ClimbConstants.rightclimbMotorID);
    leftClimbMotor = new Kraken(ClimbConstants.leftClimbMotorID);
    climbCANcoder = new CANcoder(ClimbConstants.climbCANcoderID);

    rightClimbMotor.resetEncoder();
    leftClimbMotor.resetEncoder();

    rightClimbMotor.restoreFactoryDefaults();
    leftClimbMotor.restoreFactoryDefaults();

    rightClimbMotor.addEncoder(climbCANcoder);
    leftClimbMotor.addEncoder(climbCANcoder);

    rightClimbMotor.setSoftLimits(0,0);
    leftClimbMotor.setSoftLimits(0, 0);

    rightClimbMotor.setMotorCurrentLimits(0);
    leftClimbMotor.setMotorCurrentLimits(0);

    rightClimbMotor.setInverted();
    leftClimbMotor.setNotInverted();

    rightClimbMotor.setPIDValues(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I, ClimbConstants.CLIMB_D, ClimbConstants.CLIMB_SFF, ClimbConstants.CLIMB_VFF, ClimbConstants.CLIMB_AFF);
    leftClimbMotor.setPIDValues(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I, ClimbConstants.CLIMB_D, ClimbConstants.CLIMB_SFF, ClimbConstants.CLIMB_VFF, ClimbConstants.CLIMB_AFF);

    rightClimbMotor.setBrakeMode();
    leftClimbMotor.setBrakeMode();

    desiredAngle = ClimbConstants.ClimbDesiredAngle;
    climbCANcoder.setPosition(desiredAngle);
  }

  public void stopClimb() {
    rightClimbMotor.setMotionMagicParameters(0,0,0);
    leftClimbMotor.setMotionMagicParameters(0, 0, 0);
  }

  public void setDesiredState(double angle){
    desiredAngle = angle;
  }

  public void ClimbIncrement(double increment) {
    desiredAngle += increment;
  }
  
  public void climbLeftMotorSpeed(double speed) {
     leftClimbMotor.setMotorSpeed(speed);
     rightClimbMotor.setMotorSpeed(speed);
  }

  public boolean ClimbExtended() {
     return rightClimbMotor.getPosition() > ClimbConstants.rightClimbExtended;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb at desired point", outerClimbLimitSwitch.get());
    // This method will be called once per scheduler run
  }
}
