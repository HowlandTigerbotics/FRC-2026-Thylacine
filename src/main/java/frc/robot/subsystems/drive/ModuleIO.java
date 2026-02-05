// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/**
 * <p>ModuleIO is the main abstraction class of SwerveModules</p>
 * 
 * @author Bo Kuang
 * @version 1.0
 * @since 2/5/2026
 */
public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveVelocityFilteredRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double[] turnTempCelcius = new double[] {};
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
     * @param double voltage
    */
    public default void setDriveVoltage(double volts) {}

    /** 
     * Run the turn motor at the specified voltage. 
     * 
     * @param double voltage
     */
    public default void setTurnVoltage(double volts) {}

    /** 
     * Enable or disable brake mode on the drive motor. 
     *
     * @param enable  
     */
    public default void setDriveBrakeMode(boolean enable) {}

    /** 
     * Enable or disable brake mode on the turn motor. 
     *
     * @param enable 
     */
    public default void setTurnBrakeMode(boolean enable) {}
}
