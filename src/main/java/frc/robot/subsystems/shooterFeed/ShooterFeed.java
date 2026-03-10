// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterFeed;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ShooterFeed extends SubsystemBase {
    private final ShooterFeedIO shooterFeedIO;
    private final ShooterFeedIOInputsAutoLogged shooterFeedInputs = new ShooterFeedIOInputsAutoLogged();

    private final Alert shooterFeedDisconnectedAlert = new Alert("Disconnected Shooter Feed", AlertType.kError);

    public ShooterFeed(ShooterFeedIO shooterFeedIO) {
        this.shooterFeedIO = shooterFeedIO;
    }

    @Override
    public void periodic() {
        shooterFeedIO.updateInputs(shooterFeedInputs);
        Logger.processInputs("Shooter", shooterFeedInputs);

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            shooterFeedIO.setFeedSpeed(0);
        }
    }

    public void setFeedSpeed(double speed) {
        shooterFeedIO.setFeedSpeed(speed);
    }
}
