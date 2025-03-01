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
  public static class ControllerConstants {
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

  // Elevator Constants
  public static class ElevatorConstants {
    public static final int leftMotorID = 15;
    public static final int rightMotorID = 15;

  // elevator motor PID
    public static final double ELEVATOR_P = 0.01; // ADJUST ALL
    public static final double ELEVATOR_I = 0.001;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_SFF = 0; // static feedforward
    public static final double ELEVATOR_VFF = 0; // velocity feedforward
    public static final double ELEVATOR_AFF = 0; // acceleration feedforward
    public static final double ELEVATOR_MIN_OUTPUT = -1;
    public static final double ELEVATOR_MAX_OUTPUT = 1;
  }

  // Placer Constants
  public static class PlacerConstants {
    public static final int backMotorID = 16;
    public static final int frontMotorID = 17;
  }

  // Climb Constants
  public static class ClimbConstants {
    public static final int rightclimbMotorID = 18;
    public static final int leftClimbMotorID = 19;
    public static final int climbCANcoderID = 20;
    public static final int climbCageSwitchID = 21;
    public static final int fullyRetractedLimitSwitchID = 22;

    public static final double CageContact = 0;
    public static final double ClimbFullyRetracted = 0;


  // climb motor PID
    public static final double CLIMB_P = 0.01; // ADJUST ALL
    public static final double CLIMB_I = 0.001;
    public static final double CLIMB_D = 0;
    public static final double CLIMB_SFF = 0; // static feedforward
    public static final double CLIMB_VFF = 0; // velocity feedforward
    public static final double CLIMB_AFF = 0; // acceleration feedforward
    
    public static final double CLIMB_MIN_OUTPUT = -1;
    public static final double CLIMB_MAX_OUTPUT = 1;

    public static final double ClimbOutangle = 0;
    public static final double ClimbInangle = 0;
    public static final double ClimbLock = 0;
    public static final double ClimbDesiredAngle = 0;
  }

  // Blinkin Constants
  public static class BlinkinConstants {
    public static final int blinkinID = 23;

  // light codes
    public static final double DARK_RED = 0.59; // temporary default
    public static final double OCEAN_PALETTE_WAVES = -0.41; // temporary climbing
  }

}
