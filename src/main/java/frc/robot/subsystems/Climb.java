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
  private DigitalInput cageContactLimitSwitch, fullyRetractedLimitSwitch; // fully retracted is for the "all the way out" state
  private Kraken rightClimbMotor, leftClimbMotor;
  private CANcoder climbCANcoder;
  private double desiredAngle;
  private boolean wasCageContacted;
  private boolean wasClimbRetracted;

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
    //Instantiates objects
    rightClimbMotor = new Kraken(ClimbConstants.rightClimbMotorID);
    leftClimbMotor = new Kraken(ClimbConstants.leftClimbMotorID);
    climbCANcoder = new CANcoder(ClimbConstants.climbCANCoderID);
    cageContactLimitSwitch = new DigitalInput(ClimbConstants.climbCageSwitchID);
    fullyRetractedLimitSwitch = new DigitalInput(ClimbConstants.climbFullyRetractedLimitSwitchID);

    //Resets motors
    leftClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.restoreFactoryDefaults();
    
    //Inverts left and right motors
    leftClimbMotor.setNotInverted();

    //Makes right climb motor follow the left climb motor
    rightClimbMotor.follow(ClimbConstants.leftClimbMotorID, true);

    //Resets encoder values
    leftClimbMotor.resetEncoder();

    //Creates CANcoder
    leftClimbMotor.addEncoder(climbCANcoder);

    //Motor speed limit
    //leftClimbMotor.setSoftLimits(0, 0);

    //Motor current limit
    //leftClimbMotor.setMotorCurrentLimits(0);

    //Sets PID values
    leftClimbMotor.setPIDValues(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I, ClimbConstants.CLIMB_D,
                                ClimbConstants.CLIMB_SFF, ClimbConstants.CLIMB_VFF, ClimbConstants.CLIMB_AFF, ClimbConstants.CLIMB_GFF);

    //Sets motors to break mode initially
    leftClimbMotor.setBrakeMode();
    rightClimbMotor.setBrakeMode();

    //Limit switch initial states
    wasCageContacted = cageContactLimitSwitch.get();
    SmartDashboard.putBoolean("Cage contacted", cageContactLimitSwitch.get());
    
    wasClimbRetracted = fullyRetractedLimitSwitch.get();
    SmartDashboard.putBoolean("Climb fully retracted", fullyRetractedLimitSwitch.get());

  }

    public Command SysIDQuasistatic(SysIdRoutine.Direction direction) {
    return appliedRoutine.quasistatic(direction);
  }

  public Command SysIDDynamic(SysIdRoutine.Direction direction) {
    return appliedRoutine.dynamic(direction);
  }
  
  //Speed at which the motors will go
  public void setClimbSpeed(double speed) {
     leftClimbMotor.setMotorSpeed(speed);
  }

  //Checks whether climb has made contact with cage 
  public boolean isCageContacted() {
     return cageContactLimitSwitch.get();
  }

  //Checks whether climb has been fully retracted
  public boolean isArmFullyRetracted() {
    return fullyRetractedLimitSwitch.get();
 }

  @Override
  public void periodic() {

    //Displays whether cage contact limit switch has been tripped
    boolean isCageLimitSwitchSet = cageContactLimitSwitch.get();
    if(isCageLimitSwitchSet != wasCageContacted) {
      SmartDashboard.putBoolean("Cage contacted", cageContactLimitSwitch.get());
      wasCageContacted = isCageLimitSwitchSet;
    }

    //Displays whether climb is fully retracted
    boolean isClimbFullyRetracted = fullyRetractedLimitSwitch.get();
    if(isClimbFullyRetracted != wasClimbRetracted) {
      SmartDashboard.putBoolean("Climb fully retracted", fullyRetractedLimitSwitch.get());
      wasClimbRetracted = isClimbFullyRetracted;
    }
    
    // This method will be called once per scheduler run
  }
}
