// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeRoller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO intakeIO;
  private final IntakeRollerIOInputsAutoLogged intakeInputs = new IntakeRollerIOInputsAutoLogged();
  private final Alert intakeDisconnectedAlert = new Alert("Disconnected Intake.", AlertType.kError);

  public IntakeRoller(
      IntakeRollerIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
    
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      intakeIO.setIntakeOpenLoop(0);
    }

    // Update intake alert
    intakeDisconnectedAlert.set(!intakeInputs.intakeConnected);
  }

  public void setIntakePercent(double percent) {
    intakeIO.setIntakePercent(percent);
  }

  public void stop() {
    intakeIO.setIntakeOpenLoop(0);
  }
}
