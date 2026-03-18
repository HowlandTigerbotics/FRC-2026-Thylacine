// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface HopperIO {
    @AutoLog
    public static class HopperIOInputs {
        public boolean hopperConnected = false;
        public double hopperPositionRad = 0.0;
        public double hopperVelocityRadPerSec = 0.0;
        public double hopperAppliedVolts = 0.0;
        public double hopperCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(HopperIOInputs inputs) {
    }

    /** */
    public default void setHopperPercent(double percent) {
    }
}
