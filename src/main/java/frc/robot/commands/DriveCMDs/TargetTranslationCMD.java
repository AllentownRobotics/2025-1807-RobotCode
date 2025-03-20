// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetTranslationCMD extends Command {
  Vision limelight;
  double offset;

  PIDController controller;
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;

  /** Creates a new Target. */
  public TargetTranslationCMD(Vision limelight, double offset) {
    this.limelight = limelight;
    this.offset = offset;
    
    drive = new SwerveRequest.RobotCentric()
            .withDeadband(5).withRotationalDeadband(5)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    controller = new PIDController(0.4, 0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Optional<Pose2d> pose = limelight.frontPoseTargetSpace();
    if (pose.isPresent()) {
      //controller.calculate(pose.get().getX(), offset);
      
      drivetrain.applyRequest(() ->
      drive.withVelocityY(controller.calculate(pose.get().getX(), offset))
      
      );
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
