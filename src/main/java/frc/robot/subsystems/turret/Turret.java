// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: add fancy math. Use last known angle and pos of bot
// to calc predicted position of hub.
// and add helper functions
public class Turret extends SubsystemBase {
  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  

  private final Alert turretDisconnectedAlert = 
    new Alert("Disconnected Turret", AlertType.kError);

  public Turret(TurretIO turretIO) {
    this.turretIO = turretIO;
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Turret", turretInputs);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      turretIO.setTurretOpenLoop(0);
    }

    // Update Turret Alert
    turretDisconnectedAlert.set(!turretInputs.positionMotorConnected);
  }

  public void setTurretOpenLoop(double output) {
    if (turretInputs.leftLimitHit && output > 0) {
      turretIO.setTurretPercent(0);
    }
    if (turretInputs.rightLimitHit && output < 0) {
      turretIO.setTurretPercent(0);
    }
    turretIO.setTurretOpenLoop(output);
  }

  public void goToPosition(Rotation2d rotation) {
    turretIO.setTurretPosition(rotation);
  }

  public void setTurretPercent(double percent) {
    if (turretInputs.leftLimitHit && percent > 0) {
      turretIO.setTurretPercent(0);
    }
    if (turretInputs.rightLimitHit && percent < 0) {
      turretIO.setTurretPercent(0);
    }
    turretIO.setTurretPercent(percent);
  }

  public void stop() {
    turretIO.setTurretPercent(0);
  }

  public Rotation2d getRotation() {
    return turretInputs.positionMotorPosition;
  }

  public double getVelocityRadPerSec() {
    return turretInputs.positionMotorVelocityRadPerSec;
  }

  public void setRotation(Rotation2d rotation) {
    turretInputs.positionMotorPosition = rotation;
  }

  public boolean hitLeftLimit() {
    return turretInputs.leftLimitHit;
  }
  
  public boolean hitRightLimit() {
    return turretInputs.rightLimitHit;
  }
}
