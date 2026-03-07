// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeConnected = false;
        public double intakePositionRad = 0.0;
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;

        public boolean positionConnected = false;
        public Rotation2d positionPosition = Rotation2d.kZero;
        public double positionVelocityRadPerSec = 0.0;
        public double positionAppliedVolts = 0.0;
        public double positionCurrentAmps = 0.0;

        public boolean feedConnected = false;
        public double feedPositionRad = 0.0;
        public double feedVelocityRadPerSec = 0.0;
        public double feedAppliedVolts = 0.0;
        public double feedCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {
    }

    /** Run the intake motor at the specified open loop value. */
    public default void setIntakeOpenLoop(double output) {
    }

    /** Run the position motor at the specified open loop value. */
    public default void setPositionOpenLoop(double output) {
    }

    /** Run the intake motor at the specified velocity. */
    public default void setIntakeVelocity(double velocityRadPerSec) {
    }

    /** Run the position motor to the specified rotation. */
    public default void setPositionPosition(Rotation2d rotation) {
    }

    /** Run the intake motor at the desired speed. */
    public default void setIntakeSpeed(double speed) {
    }

    /** Run the position motor at the desired speed */
    public default void setPositionSpeed(double speed) {
    }

    /** */
    public default void setFeedSpeed(double speed) {
    }
}
