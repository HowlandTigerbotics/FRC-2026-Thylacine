// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeFeed;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeFeedIO {
    @AutoLog
    public static class IntakeFeedIOInputs {
        public boolean feedConnected = false;
        public double feedPositionRad = 0.0;
        public double feedVelocityRadPerSec = 0.0;
        public double feedAppliedVolts = 0.0;
        public double feedCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeFeedIOInputs inputs) {
    }

    /** */
    public default void setFeedPercent(double percent) {
    }
}
