// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * <p>ModuleIO is the main abstraction class of SwerveModules</p>
 * 
 * @author Bo Kuang
 * @version 1.0
 * @since 2/5/2026
 */
public interface ModuleIO {
    /**
     * Wrapper Class of data that we can log and record.
     */
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
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
    }

    /**
     * Updates the set of loggable inputs. 
     * 
     * @param inputs inputs to log and update
     */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** 
     * Run the drive motor at the specified voltage. 
     * 
     * @param output voltage to run the drive motor.
    */
    public default void setDriveOpenLoop(double output) {}

    /** 
     * Run the steer motor at the specified voltage. 
     * 
     * @param output voltage to run the steer motor
     */
    public default void setSteerOpenLoop(double output) {}

    /**
     * Sets the speed of the drive motor at the specified speed.
     * 
     * @param speed
     */
    public default void setDriveVelocity(double speed) {}

    /**
     * 
     * 
     * @param rotation
     */
    public default void setSteerPosition(Rotation2d rotation) {}
}
