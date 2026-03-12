// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final Alert shooterDisconnectedAlert = 
  new Alert("Disconnected Turret", AlertType.kError);

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      shooterIO.setShooterOpenLoop(0);
    }

    shooterDisconnectedAlert.set(!shooterInputs.shooterConnected);
  }

  public void setPercent(double percent) {
    shooterIO.setShooterPercent(percent);
  }

  public void stop() {
    shooterIO.setShooterPercent(0);
  }
}
