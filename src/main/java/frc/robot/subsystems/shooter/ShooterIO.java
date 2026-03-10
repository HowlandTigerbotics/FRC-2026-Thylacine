// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean shooterConnected = false;
        public double shooterPositionRad = 0.0;
        public double shooterVelocityRadPerSec = 0.0;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {
    }

    /** */
    public default void setShooterOpenLoop(double voltage) {
    }

    /** */
    public default void setShooterVelocity(double velocity) {
    }

    /**  */
    public default void setShooterSpeed(double speed) {
    }
}
