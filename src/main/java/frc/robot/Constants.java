// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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

    public static final double driveTrainRadius = Units.inchesToMeters(Math.sqrt(Math.pow(12, 2) + Math.pow(12, 2))); // in
                                                                                                                      // meters

    public static final double PX_CONTROLLER = 5;
    public static final double P_THETA_CONTROLLER = 1;

  }
  
  // Climb Constants
  public static class ClimbConstants {
    public static final int leftClimbMotorID = 15;
    public static final int rightClimbMotorID = 16;
    public static final int climbCANCoderID = 17;

    public static final int climbCageSwitchID = 2;
    public static final int climbFullyRetractedLimitSwitchID = 3;

    public static final double CageContact = 0;
    public static final double ClimbFullyRetracted = 0;

    // climb motor PID
    public static final double CLIMB_P = 0.01; // ADJUST ALL
    public static final double CLIMB_I = 0;
    public static final double CLIMB_D = 0;
    public static final double CLIMB_SFF = 0; // static feedforward
    public static final double CLIMB_VFF = 0; // velocity feedforward
    public static final double CLIMB_AFF = 0; // acceleration feedforward
    public static final double CLIMB_GFF = 0;
    public static final double CLIMB_MIN_OUTPUT = -1;
    public static final double CLIMB_MAX_OUTPUT = 1;
    
    // climb setpoints
    public static final double climbOuterAngle = 180; // 90 degree angle outward
    public static final double climbInnerAngle = 0; // 90 degree angle inward
    public static final double climbLockAngle = 0;
    public static final double ClimbDesiredAngle = 0;
    public static final double climbSpeed = 0.5;
  }

  // Elevator Constants
  public static class ElevatorConstants {
    public static final int leftMotorID = 18;
    public static final int rightMotorID = 19;
    public static final int elevatorCANCoderID = 20;

    public static final int lowerLimitSwitchPort = 9;
    public static final int upperLimitSwitchPort = 8;

    // elevator motor PID
    public static final double ELEVATOR_P = 0.25; // ADJUST ALL
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_SFF = 0; // static feedforward
    public static final double ELEVATOR_VFF = 0; // velocity feedforward
    public static final double ELEVATOR_AFF = 0; // acceleration feedforward
    public static final double ELEVATOR_GFF = 0.01; // gravity feedforward RETUNE
    public static final double ELEVATOR_MIN_OUTPUT = -1;
    public static final double ELEVATOR_MAX_OUTPUT = 1;
    
    // elevator setpoints (inches)
    public static final double homePosition = 0;
    public static final double L1Position = 19;
    public static final double L2Position = 28;
    public static final double L3Position = 0;
    public static final double L4Position = 0;
    public static final double positionTolerance = .1;
    public static final double softLimitMinPosition = 0;
    public static final double softLimitMaxPosition = 0;
    public static final double incrementMeasurement = 5;

    public static final double elevatorGearing = 14 / 60; // inches
    public static final double elevatorSprocketRadius = 1.037;
    public static final double elevatorSprocketCircumference = 2 * Math.PI * elevatorSprocketRadius; // inches
    public static final double elevatorEncoderToMechanismRatio = (1 / elevatorSprocketCircumference) / 3;
  }
  
  // Placer Constants
  public static class PlacerConstants {
    public static final int placerFrontMotorID = 21;
    public static final int placerRearMotorID = 22;
    public static final int placerBeamBreakID = 1;
    
    public static final double placerFrontMotorSpeed = 0.2;
    public static final double placerBackMotorSpeed = 0.15; // .125
    public static final double placerAlgaeSpeed = 0.4;
    public static final double placerCollectAlgaeSpeed = 0.6;
  }
  
  // Hopper Constants
  public static class HopperConstants {
    public static final int hopperBeamBreakID = 0;
  }

  // Blinkin Constants
  public static class BlinkinConstants {
    public static final int blinkinID = 10;
    
    public enum LEDPattern {
      OFF,
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
      CLIMB_COMPLETE_RED,
      CLIMB_COMPLETE_BLUE,
      CLIMB_COMPLETE
    }

    // light codes
    public static final double defaultColor = 0.59; // dark red
    public static final double alertHumanPlayer = -0.05; // strobe white (flashing)
    public static final double climbing = -0.57; //fire large
    public static final double climbOcean = -0.95; // rainbow ocean palette
    public static final double climbLava = -0.93; // rainbow lava palette
    public static final double climbUndecided = -0.91;//rainbow forest palette
    public static final double alignedWithReef = 0.35; // strobe color 2 (green)
    public static final double elevatorAtDesiredPosition = -0.85; //shot red
    public static final double off = 0.99;//black
    public static final double idle = -0.17;//breath red
    public static final double celebrate = -0.97; //rainbow party palette}
  }

  public static class VisionConstants {
    public static final double placerOffsetToRobotCenter = 0.0;
    
    public static final double rotation_kP = 0.1;
    public static final double rotation_kI = 0.0;
    public static final double rotation_kD = 0.05;
    public static final double translation_kP = 0.75;
    public static final double translation_kI = 0.0;
    public static final double translation_kD = 0.0;
    
    public static final double rotationTargetingSpeed = 0.75; // rotations per second

    public static final double translationTargetingSpeed = 5.41; // meters per second
  }

}
