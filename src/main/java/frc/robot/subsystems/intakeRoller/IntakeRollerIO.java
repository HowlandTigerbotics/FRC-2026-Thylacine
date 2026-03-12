// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeRoller;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeRollerIO {
    @AutoLog
    public static class IntakeRollerIOInputs {
        public boolean intakeConnected = false;
        public double intakePositionRad = 0.0;
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeRollerIOInputs inputs) {
    }

    /** Run the intake motor at the specified open loop value. */
    public default void setIntakeOpenLoop(double output) {
    }

    /** Run the intake motor at the specified velocity. */
    public default void setIntakeVelocity(double velocityRadPerSec) {
    }

    /** Run the intake motor at the desired speed. */
    public default void setIntakePercent(double percent) {
    }
}
