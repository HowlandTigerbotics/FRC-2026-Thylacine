// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeFeed;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class IntakeFeed extends SubsystemBase {
  private final IntakeFeedIO intakeFeedIO;
  private final IntakeFeedIOInputsAutoLogged intakeFeedInputs = new IntakeFeedIOInputsAutoLogged();
  private final Alert intakeDisconnectedAlert = new Alert("Disconnected Intake Feed.", AlertType.kError);

  public IntakeFeed(
      IntakeFeedIO intakeFeedIO) {
    this.intakeFeedIO = intakeFeedIO;
  }

  @Override
  public void periodic() {
    intakeFeedIO.updateInputs(intakeFeedInputs);
    Logger.processInputs("IntakeFeed", intakeFeedInputs);
    
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      intakeFeedIO.setFeedPercent(0);
    }

    // Update intake alert
    intakeDisconnectedAlert.set(!intakeFeedInputs.feedConnected && Constants.currentMode != Mode.SIM);
  }

  public void stop() {
    intakeFeedIO.setFeedPercent(0);
  }

  public void setFeedPercent(double percent) {
    intakeFeedIO.setFeedPercent(percent);
  }
}
