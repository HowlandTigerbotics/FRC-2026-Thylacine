// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakePosition;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePosition extends SubsystemBase {
  private final IntakePositionIO intakePositionIO;
  private final IntakePositionIOInputsAutoLogged intakePositionInputs = new IntakePositionIOInputsAutoLogged();
  private final Alert intakeDisconnectedAlert = new Alert("Disconnected Intake Position.", AlertType.kError);

  public IntakePosition(
      IntakePositionIO intakePositionIO) {
    this.intakePositionIO = intakePositionIO;
  }

  @Override
  public void periodic() {
    intakePositionIO.updateInputs(intakePositionInputs);
    Logger.processInputs("IntakePosition", intakePositionInputs);
    
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      intakePositionIO.setPositionOpenLoop(0);
    }

    // Update intake alert
    intakeDisconnectedAlert.set(!intakePositionInputs.positionConnected);
  }

  public void stop() {
    intakePositionIO.setPositionOpenLoop(0);
  }

  public void setPositionPercent(double percent) {
    intakePositionIO.setPositionPercent(percent);
  }
}
