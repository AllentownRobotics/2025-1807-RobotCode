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
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.BlinkinConstants.LEDPattern;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetCMD extends Command {
  Vision limelight;
  double offset;
  Blinkin blinkin;

  PIDController sideToSideController;
  PIDController frontToBackController;
  PIDController rotationController;
  CommandXboxController driverController;

  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  SwerveRequest.FieldCentric driveFieldRelative;

  double slowDriveSpeed;
  double slowAngularRate;

  /** Creates a new TargetCMD. */
  public TargetCMD(Vision limelight, CommandSwerveDrivetrain drivetrain, Blinkin blinkin, CommandXboxController driverController, double offset) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.offset = offset;
    this.blinkin = blinkin;
    
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

    sideToSideController = new PIDController(VisionConstants.translation_kP, VisionConstants.translation_kI, VisionConstants.translation_kD);
    frontToBackController = new PIDController(VisionConstants.translation_kP, VisionConstants.translation_kI, VisionConstants.translation_kD);
    rotationController = new PIDController(VisionConstants.rotation_kP, VisionConstants.rotation_kI, VisionConstants.rotation_kD);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sideToSideController.reset();
    frontToBackController.reset();
    rotationController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Optional<Pose2d> pose = limelight.frontPoseTargetSpace();
    if (pose.isPresent()) {
      SmartDashboard.putNumber("Optional translation Pose", pose.get().getX());
      SmartDashboard.putNumber("Optional rotation Pose", pose.get().getRotation().getRadians());
      SmartDashboard.putNumber("optional back front translation pose", pose.get().getY());
      SmartDashboard.putNumber("PID front back translation value", frontToBackController.calculate(pose.get().getY(), VisionConstants.targetingFrontBackTranslationOffset));
      SmartDashboard.putNumber("PID left right translation value", sideToSideController.calculate(pose.get().getX(), offset));
      SmartDashboard.putNumber("PID rotation", rotationController.calculate(pose.get().getRotation().getRadians(), 0));
    
      drivetrain.applyRequest(() ->
        drive
        .withVelocityX(frontToBackController.calculate(pose.get().getY(), VisionConstants.targetingFrontBackTranslationOffset))
        .withVelocityY(-sideToSideController.calculate(pose.get().getX(), offset))
        .withRotationalRate(-rotationController.calculate(pose.get().getRotation().getRadians(), 0))
      ).execute();

      if(Math.abs(offset - pose.get().getX()) <= 0.05) {
        blinkin.setPattern(LEDPattern.ALIGNED_WITH_REEF);
      }

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
