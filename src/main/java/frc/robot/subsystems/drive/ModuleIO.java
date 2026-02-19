// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.


package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * <p>
 * ModuleIO is the main abstraction class of SwerveModules
 * </p>
 * 
 * @author Mechanical Advantage (6328)
 * @author Bo Kuang (8718)
 * @version 1.0
 * @since 2/5/2026
 */
public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean steerConnected = false;
        public Rotation2d steerPosition = Rotation2d.kZero;
        public double steerVelocityRadPerSec = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {
    }

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {
    }

    /** Run the steer motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {
    }

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec) {
    }

    /** Run the steer motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {
    }
}