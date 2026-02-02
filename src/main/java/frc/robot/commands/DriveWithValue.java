// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveWithJoysticks.JoystickMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithValue extends Command {
  /** Creates a new DriveWithValue. */
  private final Drive drive;
  private final double leftX;
  private final double leftY;
  private final double rightY;
  private final boolean robotRelativeOverride;
  private final JoystickMode modeSupplier;
  private final double linearSpeedLimitSupplier;
  private final double angularSpeedLimitSupplier;
  private final double autoDriveSupplier;

  private static final double deadband = 0.1;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithValue(Drive drive, double leftXSupplier,
      double leftYSupplier, double rightYSupplier,
      boolean robotRelativeOverride,
      JoystickMode modeSupplier,
      double linearSpeedLimitSupplier,
      double angularSpeedLimitSupplier,
      double autoDriveSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.leftX = leftXSupplier;
        this.leftY = leftYSupplier;
        this.rightY = rightYSupplier;
        this.robotRelativeOverride = robotRelativeOverride;
        this.modeSupplier = modeSupplier;
        this.linearSpeedLimitSupplier = linearSpeedLimitSupplier;
        this.angularSpeedLimitSupplier = angularSpeedLimitSupplier;
        this.autoDriveSupplier = autoDriveSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = -this.leftX;
    double leftY = -this.leftY;
    double rightY = -this.rightY;

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
    rightY = MathUtil.applyDeadband(rightY, deadband);

    // Apply squaring
    linearMagnitude =
        Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightY = Math.copySign(rightY * rightY, rightY);

    // Apply speed limits
    linearMagnitude *= linearSpeedLimitSupplier;
    rightY *= angularSpeedLimitSupplier;

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(
                GeomUtil.transformFromTranslation(linearMagnitude, 0.0))
            .getTranslation();
    if (modeSupplier == JoystickMode.Tank) {
      linearVelocity = new Translation2d(linearVelocity.getX(), 0.0);
    }

    // Convert to meters per second
    ChassisSpeeds speeds = new ChassisSpeeds(
        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        rightY * drive.getMaxAngularSpeedRadPerSec());

    // Convert from field relative
    if (!robotRelativeOverride
        && modeSupplier == JoystickMode.Standard) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
          drive.getRotation());
    }

    // Apply auto drive
    // ChassisSpeeds autoDriveSpeeds =
    // AutoDriveSoftWithSpline.calculate(drive.getPose(),
    // autoDriveSupplier.get() * drive.getMaxLinearSpeedMetersPerSec(),
    // autoDriveSupplier.get() > 0.08);
    // speeds = new ChassisSpeeds(
    // speeds.vxMetersPerSecond + autoDriveSpeeds.vxMetersPerSecond,
    // speeds.vyMetersPerSecond + autoDriveSpeeds.vyMetersPerSecond,
    // speeds.omegaRadiansPerSecond + autoDriveSpeeds.omegaRadiansPerSecond);

    // Send to drive
    drive.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
