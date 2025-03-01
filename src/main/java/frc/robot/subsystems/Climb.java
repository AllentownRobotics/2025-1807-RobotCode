// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ClimbConstants;
import frc.utils.Kraken;

public class Climb extends SubsystemBase {

  DigitalInput outerClimbLimitSwitch, innerClimbLimitSwitch;
  Kraken leftClimbMotor, rightClimbMotor;
  CANcoder climbEncoder;
  double desiredAngle;

    private final SysIdRoutine climbSysIDRoutine = new SysIdRoutine(
    new Config(
      null,
      Voltage.ofBaseUnits(4, Volts),
      null),
    new Mechanism(
      state -> SignalLogger.writeString("sysID State", state.toString()),
      null,
      this));

  private SysIdRoutine appliedRoutine = climbSysIDRoutine;


  /** Creates a new Climb. */
  public Climb() {
    
    leftClimbMotor = new Kraken(ClimbConstants.leftClimbMotorID);
    rightClimbMotor = new Kraken(ClimbConstants.rightClimbMotorID);
    climbEncoder = new CANcoder(ClimbConstants.climbCANCoderID);

    leftClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.restoreFactoryDefaults();

    rightClimbMotor.follow(ClimbConstants.leftClimbMotorID, true);

    leftClimbMotor.addEncoder(climbEncoder);

    leftClimbMotor.setPIDValues(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I,
                                  ClimbConstants.CLIMB_D, ClimbConstants.CLIMB_SFF,
                                  ClimbConstants.CLIMB_VFF, ClimbConstants.CLIMB_AFF);

    leftClimbMotor.setBrakeMode();
    leftClimbMotor.setNotInverted();

    leftClimbMotor.setMotorCurrentLimits(40);
    leftClimbMotor.setSoftLimits(ClimbConstants.climbInnerAngle, ClimbConstants.climbLockAngle); 
                                //prevents motor from extending past the lock angle

    desiredAngle = ClimbConstants.climbInnerAngle;
    climbEncoder.setPosition(desiredAngle);
  }

    public Command SysIDQuasistatic(SysIdRoutine.Direction direction) {
    return appliedRoutine.quasistatic(direction);
  }

  public Command SysIDDynamic(SysIdRoutine.Direction direction) {
    return appliedRoutine.dynamic(direction);
  }

  public void stopClimb() {
    leftClimbMotor.stopMotor();
  }

  public void setDesiredAngle(double angle){
    desiredAngle = angle;
    climbEncoder.setPosition(desiredAngle);
  }

  public void incrementDesiredAngle(double increment) {
    desiredAngle += increment;
    climbEncoder.setPosition(desiredAngle);
  }

  // look at new repo
  public boolean isClimbExtended() {
     return rightClimbMotor.getPosition() > ClimbConstants.climbInnerAngle;
  }

  public boolean innerLimitSwitchStatus() {
    return innerClimbLimitSwitch.get();
  }

  public boolean outerLimitSwitchStatus() {
    return outerClimbLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Tells you when the climb is at setpoint
    if(outerLimitSwitchStatus() != true) { //change true to a boolean set to false
      leftClimbMotor.setMotorSpeed(-.2); //replace these numbers to spin the motors away from the limit switch
      SmartDashboard.putBoolean("Climb at setpoint", outerLimitSwitchStatus());
    }

    if(innerLimitSwitchStatus() == true) {
      leftClimbMotor.setMotorSpeed(.2); //replace these numbers to spin the motors away from the limit switch
    }

  }
}
