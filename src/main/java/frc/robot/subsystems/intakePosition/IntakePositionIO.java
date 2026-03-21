// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakePosition;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface IntakePositionIO {
    @AutoLog
    public static class IntakePositionIOInputs {
        public boolean positionConnected = false;
        public Rotation2d positionPosition = Rotation2d.kZero;
        public double positionVelocityRadPerSec = 0.0;
        public double positionAppliedVolts = 0.0;
        public double positionCurrentAmps = 0.0;

        public boolean limitSwitch = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakePositionIOInputs inputs) {
    }

    /** Run the position motor at the specified open loop value. */
    public default void setPositionOpenLoop(double output) {
    }

    /** Run the position motor to the specified rotation. */
    public default void setPositionPosition(Rotation2d rotation) {
    }

    /** Run the position motor at the desired speed */
    public default void setPositionPercent(double percent) {
    }
}
