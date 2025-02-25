// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbIn;
import frc.robot.commands.ClimbOut;
import frc.robot.subsystems.Climb;


public class RobotContainer {
    //private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final Climb climbSubsystem = new Climb();


    private final CommandXboxController driverController = new CommandXboxController(0);


    public RobotContainer() {
        
        configureBindings();
    }

    private void configureBindings() {
        driverController.leftBumper().whileTrue(new ClimbOut(climbSubsystem));
        driverController.rightBumper().whileTrue(new ClimbIn(climbSubsystem));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
