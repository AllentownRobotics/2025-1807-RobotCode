// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TargetingCMDs;

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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetCMD extends Command {

  Vision limelight;
  double leftRightOffset;
  double frontBackOffset;
  
  PIDController sideToSideController;
  PIDController frontToBackController;
  PIDController rotationController;
  CommandXboxController driverController;

  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  SwerveRequest.FieldCentric driveFieldRelative;

  double slowDriveSpeed;
  double slowAngularRate;

  static Pose2d previousPose = null;

  /** Creates a new TargetCMD. */
  public TargetCMD(Vision limelight, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController,
                    double leftRightOffset, double frontBackOffset) {

    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.leftRightOffset = leftRightOffset;
    this.frontBackOffset = frontBackOffset;
    
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
    addRequirements(drivetrain);
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
      previousPose = pose.get();
      
      double frontToBackCalculation = frontToBackController.calculate(pose.get().getY(), frontBackOffset);
      double sideToSideCalculation = sideToSideController.calculate(pose.get().getX(), leftRightOffset);
      double rotationCalculation = rotationController.calculate(pose.get().getRotation().getRadians(), 0);
      
      SmartDashboard.putString("is using previous pose?", "no");

      SmartDashboard.putNumber("left right translation Pose", pose.get().getX());
      SmartDashboard.putNumber("rotation Pose", pose.get().getRotation().getRadians());
      SmartDashboard.putNumber("front back translation pose", pose.get().getY());
      SmartDashboard.putNumber("PID front back translation value", frontToBackCalculation);
      SmartDashboard.putNumber("PID left right translation value", sideToSideCalculation);
      SmartDashboard.putNumber("targeting PID rotation", rotationCalculation);
    
      drivetrain.applyRequest(() ->
        drive
        .withVelocityX(frontToBackCalculation)
        .withVelocityY(-sideToSideCalculation)
        .withRotationalRate(-rotationCalculation)
      ).execute();


        } else {
      
          if(previousPose != null) {
          SmartDashboard.putString("is using previous pose?", "yes");
          
          drivetrain.applyRequest(() ->
          drive
          .withVelocityX(frontToBackController.calculate(previousPose.getY(), frontBackOffset))
          .withVelocityY(-sideToSideController.calculate(previousPose.getX(), leftRightOffset))
          .withRotationalRate(-rotationController.calculate(previousPose.getRotation().getRadians(), 0))
            //-rotationController.calculate(pose.get().getRotation().getRadians(), 0))
        ).execute();

          }
          else {
            
          }
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
