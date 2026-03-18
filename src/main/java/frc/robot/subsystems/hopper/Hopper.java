// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;
import frc.robot.subsystems.intakeFeed.IntakeFeedIO;
import frc.robot.subsystems.intakeFeed.IntakeFeedIOInputsAutoLogged;

/** Add your docs here. */
public class Hopper extends SubsystemBase {
    private final HopperIO hopperIO;
  private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();
  private final Alert hopperDisconnectedAlert = new Alert("Disconnected Intake Feed.", AlertType.kError);

  public Hopper(
      HopperIO hopperIO) {
    this.hopperIO = hopperIO;
  }

  @Override
  public void periodic() {
    hopperIO.updateInputs(hopperInputs);
    Logger.processInputs("Hopper", hopperInputs);
    
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      hopperIO.setHopperPercent(0);
    }

    // Update intake alert
    hopperDisconnectedAlert.set(!hopperInputs.hopperConnected && Constants.currentMode != Mode.SIM);
  }

  public void stop() {
    hopperIO.setHopperPercent(0);
  }

  public void setHopperPercent(double percent) {
    hopperIO.setHopperPercent(percent);
  }
}
