// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Controllers
  public static class OIConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
  }

  public static class GlobalConstants {
    // power distribution hub
    public static final int pdhID = 13;
    // Gyro ID
    public static final int pigeonID = 14;
  }

  // Auto Constants
  public static class AutoConstants { // ADJUST ALL
    public static final double AUTO_MAX_SPEED_MPS = 5;

    public static final double driveTrainRadius = Units.inchesToMeters(Math.sqrt(Math.pow(12, 2) + Math.pow(12, 2))); // in meters

    public static final double PX_CONTROLLER = 5;
    public static final double P_THETA_CONTROLLER = 1;
  }
  
  // Climb Constants
  public static class ClimbConstants {
    public static final int leftClimbMotorID = 15;
    public static final int rightClimbMotorID = 16;
    public static final int climbCANCoderID = 17;
    public static final int climbCageSwitchID = 21;
    public static final int fullyRetractedLimitSwitchID = 22;

    public static final double CageContact = 0;
    public static final double ClimbFullyRetracted = 0;

    public static final int outerLimitSwitchID = 3;
    public static final int innerLimitSwitchID = 2;

  // climb motor PID
    public static final double CLIMB_P = 0.01; // ADJUST ALL
    public static final double CLIMB_I = 0.001;
    public static final double CLIMB_D = 0;
    public static final double CLIMB_SFF = 0; // static feedforward
    public static final double CLIMB_VFF = 0; // velocity feedforward
    public static final double CLIMB_AFF = 0; // acceleration feedforward
    public static final double CLIMB_MIN_OUTPUT = -1;
    public static final double CLIMB_MAX_OUTPUT = 1;

  // climb setpoints
    public static final double climbOuterAngle = 180; // 90 degree angle outward
    public static final double climbInnerAngle = 0; // 90 degree angle inward
    public static final double climbLockAngle = 0; 
    public static final double ClimbDesiredAngle = 0;

    public static final double climbRotorToSensorRatio = 50/1;
    public static final double climbSensorToMechanismRatio = 1;
  }

  // Elevator Constants
  public static class ElevatorConstants {
    public static final int leftMotorID = 18;
    public static final int rightMotorID = 19;
    public static final int elevatorCANCoderID = 20;

    public static final int lowerLimitSwitchPort = 9;
    public static final int upperLimitSwitchPort = 8;

  // elevator motor PID
    public static final double ELEVATOR_P = 0.01; // ADJUST ALL
    public static final double ELEVATOR_I = 0.001;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_SFF = 0; // static feedforward
    public static final double ELEVATOR_VFF = 0; // velocity feedforward
    public static final double ELEVATOR_AFF = 0; // acceleration feedforward
    public static final double ELEVATOR_MIN_OUTPUT = -1;
    public static final double ELEVATOR_MAX_OUTPUT = 1;

  // elevator setpoints
    public static final double homePosition = 0;
    public static final double L1Position = 0;
    public static final double L2Position = 0;
    public static final double L3Position = 0;
    public static final double L4Position = 0;
    public static final double softLimitMinPosition = 0;
    public static final double softLimitMaxPosition = 0;
    public static final double elevatorIncrementMeasurement = 1;

    public static final double elevatorGearing = 4.25; // inches
    public static final double elevatorSprocketRadius = 1.037;
    public static final double elevatorSprocketCircumference = 2*Math.PI*elevatorSprocketRadius; // inches

    public static final double carriageMass = 12.6;
    public static final double minHeightMeters = 0;
    public static final double maxHeightMeters = 71.25; // inches
    public static final double startingHeightMeters = 0;


  }

  // Placer Constants
  public static class PlacerConstants {
    public static final int placerFrontMotorID = 21;
    public static final int placerRearMotorID = 22;

    public static final int placerBeamBreakID = 1;
  }

  // Hopper Constants
  public static class HopperConstants {
    public static final int hopperBeamBreakID = 0;
  }

  // Blinkin Constants
  public static class BlinkinConstants {
    public static final int blinkinID = 23;

  enum blinkinPattern {
    IDLE,
    ALERT_HUMAN_PLAYER,
    ALIGNED_WITH_REEF,
    CELEBRATE,
    CORAL_COLLECTED,
    CORAL_INDEXED,
    CORAL_PLACING,
    CORAL_PLACED,
    ELEVATOR_AT_DESIRED_POSITION,
    CLIMBING,
    CLIMB_COMPLETE
  }

  // light codes
  public static final double defaultColor = 0.59; // dark red
  public static final double humanPlayerStation = -0.05; // strobe white (flashing)
  public static final double climbOcean = -0.95; // rainbow ocean palette
  public static final double climbLava = -0.93; // rainbow lava palette
  public static final double alignedWithReef = 0.35; // strobe color 2 (green)
  }

}
