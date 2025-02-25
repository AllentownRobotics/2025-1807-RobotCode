// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class limlightCoordinateSystemConstants {
    public static final int xPosition = 0;
    public static final int yPosition = 1;
    public static final int zPosition = 2;
    public static final int pitch = 3;
    public static final int yaw = 4;
    public static final int roll = 5;
  }

  public static class allignmentValues {
    //All values are translations and rotations from the robot to the center of the neerest AprilTag.

    public static final double leftRodAllignmentX = -0.3302;
    public static final double leftRodAllignmentZ = 0.432308;
    public static final double leftRodAllignmentYaw = 0;

    public static final double rightRodAllignmentX = 0.0127;
    public static final double rightRodAllignmentZ = 0.432308;
    public static final double rightRodAllignmentYaw = 0;

    public static final double leftCoralStationAllignmentX = -0.6604;
    public static final double leftCoralStationAllignmentZ = 0.439801;
    public static final double leftCoralStationAllignmentYaw = 0;

    public static final double centerCoralStationAllignmentX = 0;
    public static final double centerCoralStationAllignmentZ = 0.439801;
    public static final double centerCoralStationAllignmentYaw = 0;

    public static final double rightCoralStationAllignmentX = 0.6096;
    public static final double rightCoralStationAllignmentZ = 0.439801;
    public static final double rightCoralStationAllignmentYaw = 0;
  }

  public static class telementeryValues {
    public static double MaxSpeed = telementeryValues.kSpeedAt12Volts.in(MetersPerSecond);
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.41);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final int operatorControllerPort = 1;
  }

  public static class PlacerConstants {
    public static final int placerRearMotorID = 0;

    public static final int placerFrontMotorID = 0;

    public static final int placerBeamBreakID = 0;

    public static final double placerMotorSpeed = .4;
  }

  public static class HopperConstants {
    public static final int kBeamBreakPort = 9; // Robo rio DIO port 9 
  }
}
