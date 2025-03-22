// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetCMD extends Command {
  Vision limelight;
  double offset;

  PIDController controller;
  PIDController rotationController;
  CommandXboxController driverController;

  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  SwerveRequest.FieldCentric driveFieldRelative;

  double slowDriveSpeed;
  double slowAngularRate;

  /** Creates a new TargetCMD. */
  public TargetCMD(Vision limelight, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, double offset) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.offset = offset;
    
    double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    slowDriveSpeed = MaxSpeed * TunerConstants.slowDriveScalingConstant;
    slowAngularRate = MaxAngularRate * TunerConstants.slowDriveScalingConstant;


    drive = new SwerveRequest.RobotCentric()
            .withDeadband(0.05).withRotationalDeadband(0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    driveFieldRelative = new SwerveRequest.FieldCentric()
            .withDeadband(slowDriveSpeed * 0.1)
            .withRotationalDeadband(slowAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    controller = new PIDController(3, 0.2, 0.625);
    rotationController = new PIDController(2.27, 0, 0.35);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Optional<Pose2d> pose = limelight.frontPoseTargetSpace();
    if (pose.isPresent()) {
      SmartDashboard.putNumber("Optional translation Pose", pose.get().getX());
      SmartDashboard.putNumber("Optional rotation Pose", pose.get().getRotation().getRadians());
      SmartDashboard.putNumber("PID translation Value", controller.calculate(pose.get().getX(), offset));
      SmartDashboard.putNumber("PID rotation", rotationController.calculate(pose.get().getRotation().getRadians(), 0));
      
      drivetrain.applyRequest(() ->
        drive
        .withVelocityY(-controller.calculate(pose.get().getX(), offset))
        .withRotationalRate(-rotationController.calculate(pose.get().getRotation().getRadians(), 0))
      ).execute();

        } else {
      drivetrain.applyRequest(() ->
          driveFieldRelative.withVelocityX(-driverController.getLeftY() * slowDriveSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-driverController.getLeftX() * slowDriveSpeed) // Drive left with negative X (left)
          .withRotationalRate(-driverController.getRightX() * slowAngularRate) // Drive counterclockwise with negative X (left)
      ).execute();


    }

    SmartDashboard.putBoolean("has pose", pose.isPresent());

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
