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
    public static final int testControllerPort = 5;
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

    public static final int climbCageSwitchID = 2;
    public static final int climbFullyRetractedLimitSwitchID = 3;

    public static final double CageContact = 0;
    public static final double ClimbFullyRetracted = 0;
    
    public static final double climbSpeed = 1;
    
    // climb motor PID
    // public static final double CLIMB_P = 0.01;
    // public static final double CLIMB_I = 0;
    // public static final double CLIMB_D = 0;
    // public static final double CLIMB_SFF = 0;
    // public static final double CLIMB_VFF = 0;
    // public static final double CLIMB_AFF = 0;
    // public static final double CLIMB_GFF = 0;
    // public static final double CLIMB_MIN_OUTPUT = -1;
    // public static final double CLIMB_MAX_OUTPUT = 1;
    
    // // climb setpoints
    // public static final double climbOuterAngle = 180; // 90 degree angle outward
    // public static final double climbInnerAngle = 0; // 90 degree angle inward
    // public static final double climbLockAngle = 0;
    // public static final double ClimbDesiredAngle = 0;
  }

  // Elevator Constants
  public static class ElevatorConstants {
    public static final int leftMotorID = 18;
    public static final int rightMotorID = 19;
    public static final int elevatorCANCoderID = 17;

    public static final int lowerLimitSwitchPort = 9;
    public static final int upperLimitSwitchPort = 8;

    // elevator motor PID
    public static final double ELEVATOR_P = 0.15;
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_SFF = 0; // static feedforward
    public static final double ELEVATOR_VFF = 0; // velocity feedforward
    public static final double ELEVATOR_AFF = 0; // acceleration feedforward
    public static final double ELEVATOR_GFF = 0.296; // gravity feedforward 0.296
    public static final double ELEVATOR_MIN_OUTPUT = -1;
    public static final double ELEVATOR_MAX_OUTPUT = 1;
    
    // elevator setpoints (inches)
    public static final double homePosition = -2;
    public static final double L1Position = 19;
    public static final double L2Position = 31;
    public static final double L3Position = 44;
    public static final double L4Position = 67;
    public static final double positionTolerance = .1;
    public static final double softLimitMinPosition = 0;
    public static final double softLimitMaxPosition = 0;
    public static final double incrementMeasurement = 1.5;

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
    
    public static final double placerFrontMotorSpeed = 0.2; // .2
    public static final double placerBackMotorSpeed = 0.1; // .17 //.12
    public static final double placerAlgaeSpeed = 0.4;
    public static final double placerCollectAlgaeSpeed = 0.6;
  }
  
  // Hopper Constants
  public static class HopperConstants {
    public static final int hopperBeamBreakID = 0;
  }


  // Algae Mech Constants
  public static class AlgaeMechConstants {
    public static final int collectorMotorID = 23;
    public static final int pivotMotorID = 24;
    public static final int algaeMechCANCoderID = 25;

    public static final double collectSpeed = 0.2;
    public static final double pivotSpeed = 0.4;
  }

  // Blinkin Constants
  public static class BlinkinConstants {
    public static final int blinkinID = 0;
    
    public enum LEDPattern {
      OFF(off),
      IDLE(defaultColor),
      ALERT_HUMAN_PLAYER(alertHumanPlayer),
      ALIGNED_WITH_REEF(alignedWithReef),
      CELEBRATE(celebrate),
      CORAL_INDEXED(indexedCoral),
      ELEVATOR_AT_DESIRED_POSITION(elevatorAtDesiredPosition),
      CLIMB_COMPLETE(off),
      CLIMB_COMPLETE_RED(climbLava),
      CLIMB_COMPLETE_BLUE(climbOcean);

      public final double value;
      private LEDPattern(double val){
        value = val;
      }
    }

    // light codes
    public static final double defaultColor = 0.59; // dark red
    public static final double alertHumanPlayer = -0.05; // strobe white (flashing)
    public static final double climbing = -0.57; //fire large
    public static final double climbOcean = -0.95; // rainbow ocean palette
    public static final double climbLava = -0.93; // rainbow lava palette
    public static final double climbUndecided = -0.91;//rainbow forest palette
    public static final double alignedWithReef = 0.77; // color 2 (green)
    public static final double elevatorAtDesiredPosition = -0.85; //shot red
    public static final double off = 0.99;//black
    public static final double idle = -0.17;//breath red
    public static final double celebrate = -0.97; //rainbow party palette}
    public static final double indexedCoral = 0.93;
  }

  public static class VisionConstants {

    public static final double targetingLeftTranslationOffset = -0.4 + 0.05;
    public static final double targetingRightTranslationOffset = -0.01 - 0.01 + 0.01;
    public static final double targetingBackTranslationOffset = 0.12;

    public static final double targetingLeftFrontBackTranslationOffset = -0.45;
    public static final double targetingRightFrontBackTranslationOffset = -0.38;
    public static final double targetingBackFrontBackTranslationOffset = -0.6;
    
    public static final double rotation_kP = 2.7; //2.4
    public static final double rotation_kI = 0.0;
    public static final double rotation_kD = 0; //.35

    public static final double translation_kP = 3.3;
    public static final double translation_kI = 0; //0.2
    public static final double translation_kD = 0; //0.625

    public static final double ytranslation_kP = 2.15;
    public static final double ytranslation_kI = 0; //0.2
    public static final double ytranslation_kD = 0; //0.625
    
    public static final double rotationTargetingSpeed = 0.75; // rotations per second

    public static final double translationTargetingSpeed = 5.41; // meters per second

    public static final double xDistanceDeadzone = 0.03;
    public static final double yLeftReefDistanceDeadzone = 0.06;
    public static final double yRightReefDistanceDeadzone = 0.1;
    public static final double angleDeadzone = 1.5;


    // limelight configs (center of lens) relative to center bottom of the robot (bottom of the wheels)
    // camera view: robot pose in target space

    // forward, right, and up are in meters
    // roll, pitch, and yaw are in degrees

    public static final double hopperLLForward = 0.0466188298;
    public static final double hopperLLRight =  0.2996714606;
    public static final double hopperLLUp = 0.7356746526;
    public static final double hopperLLRoll = 0;
    public static final double hopperLLPitch = -29;
    public static final double hopperLLYaw = -4.372;

    public static final double frontLLForward = 0.1637538; //0.168
    public static final double frontLLRight = -0.2289857498; //-0.234
    public static final double frontLLUp = 0.3057553194; //0.306
    public static final double frontLLRoll = 0;
    public static final double frontLLPitch = 0;
    public static final double frontLLYaw = -42.5;

    public static final double backLLForward = -0.169; //-0.1616859098;
    public static final double backLLRight = -0.0254; //0.0248326148;
    public static final double backLLUp = 1.0301; //1.0262344474;
    public static final double backLLRoll = 0;
    public static final double backLLPitch = 47;
    public static final double backLLYaw = -180;

    // limelight exposure: 351 (mar)

  }

}
